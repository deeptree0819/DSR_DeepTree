# -*- coding: utf-8 -*-
# Isaac Sim + MoveIt2 연동 스크립트 — Doosan m0609 + OnRobot RG2
#
# 전체 구조:
#   이 스크립트는 Isaac Sim(물리 시뮬레이터) 안에서 두산 m0609 로봇 암과
#   OnRobot RG2 그리퍼를 불러오고, ROS2 토픽을 통해 MoveIt2(모션 플래닝)와
#   실시간으로 연동합니다.
#
#   Isaac Sim → /isaac_joint_states 토픽 발행 → MoveIt2가 현재 관절 상태 수신
#   MoveIt2   → /isaac_joint_commands 토픽 발행 → Isaac Sim이 목표 관절 위치 수신
#
# 실행 방법:
#   터미널 A (Isaac Sim):
#     source /opt/ros/humble/setup.bash
#     source /home/ssu/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
#     source /home/ssu/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
#     /home/ssu/dev_ws/issac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
#       /home/ssu/dev_ws/issac_sim/src/isaac_moveit_m0609_rg2.py
#
#   터미널 B (MoveIt2):
#     source /opt/ros/humble/setup.bash
#     source /home/ssu/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
#     source /home/ssu/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash
#     ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py

import sys
import os
import tempfile          # 통합 URDF를 임시 파일로 저장하기 위해 사용
import numpy as np
import xml.etree.ElementTree as ET  # URDF(XML 형식)를 파싱·수정하기 위해 사용

# ─── Isaac Sim 애플리케이션 초기화 ───────────────────────────────────────────
# SimulationApp은 Isaac Sim GUI 창을 여는 진입점입니다.
# 이 객체를 생성하기 전에는 Isaac Sim의 어떤 모듈도 임포트할 수 없습니다.
# (내부적으로 Omniverse Kit 런타임을 초기화하기 때문)
try:
    from isaacsim import SimulationApp           # Isaac Sim 4.x 이상
except ImportError:
    from omni.isaac.kit import SimulationApp     # Isaac Sim 2023.x 이하 (레거시)

# headless=False: GUI 창을 표시합니다. True로 바꾸면 창 없이 실행됩니다.
CONFIG = {"renderer": "RayTracedLighting", "headless": False}
simulation_app = SimulationApp(CONFIG)

# ─── Isaac Sim 버전 확인 ──────────────────────────────────────────────────────
# Isaac Sim은 버전마다 모듈 경로가 크게 달라집니다.
#   4.5 이상: isaacsim.* 네임스페이스 사용
#   4.5 미만: omni.isaac.* 네임스페이스 사용 (레거시)
# 이 플래그로 이후 임포트 분기를 처리합니다.
isaac_sim_ge_4_5_version = True
try:
    from isaacsim.core.version import get_version
except ImportError:
    from omni.isaac.version import get_version
    isaac_sim_ge_4_5_version = False

# get_version()[2]의 길이가 4이면 구형(레거시) Isaac Sim입니다.
# 레거시 버전은 ArticulationController 사용 방식이 일부 다릅니다.
is_legacy_isaacsim = len(get_version()[2]) == 4
print(f"Isaac Sim >= 4.5: {isaac_sim_ge_4_5_version}, legacy: {is_legacy_isaacsim}")

# ─── 핵심 모듈 임포트 ─────────────────────────────────────────────────────────
# SimulationContext  : 물리 시뮬레이션(재생/정지/스텝)을 제어합니다.
# set_targets        : OmniGraph 노드의 prim 타겟 속성을 설정합니다.
# extensions         : Omniverse 확장 기능을 런타임에 활성화합니다.
# stage              : USD 스테이지(씬 트리)에 접근합니다.
# viewports          : 뷰포트 카메라 위치를 설정합니다.
# nucleus            : Isaac Sim 내장 에셋 서버(Nucleus)에 접근합니다.
try:
    from isaacsim.core.api import SimulationContext
    from isaacsim.core.utils.prims import set_targets
    from isaacsim.core.utils import extensions, stage, viewports
    from isaacsim.storage.native import nucleus
except ImportError:
    from omni.isaac.core import SimulationContext
    from omni.isaac.core.utils.prims import set_targets
    from omni.isaac.core.utils import extensions, stage, viewports
    from omni.isaac.core.utils import nucleus

from pxr import Gf                  # USD 수학 타입 (벡터, 행렬 등)
import omni.graph.core as og        # OmniGraph: 노드 기반 실행 그래프 (ROS2 브릿지의 핵심)
import omni.kit.commands            # Isaac Sim 명령 실행기 (URDF 임포트 등)
import carb                         # Omniverse 로깅 유틸리티

# ─── URDF Importer 확장 기능 활성화 ──────────────────────────────────────────
# URDF(로봇 기술 파일)를 Isaac Sim USD 씬으로 변환하는 확장 기능입니다.
# 이 확장을 활성화해야 "URDFParseAndImportFile" 명령을 쓸 수 있습니다.
if isaac_sim_ge_4_5_version:
    extensions.enable_extension("isaacsim.asset.importer.urdf")
    from isaacsim.asset.importer.urdf import _urdf
else:
    extensions.enable_extension("omni.isaac.urdf")
    from omni.isaac.urdf import _urdf

# ─── ROS2 브릿지 확장 기능 활성화 ────────────────────────────────────────────
# Isaac Sim과 ROS2 간 토픽 발행/구독을 가능하게 하는 핵심 확장입니다.
# 활성화 후 OmniGraph에서 ROS2PublishJointState 등의 노드를 사용할 수 있습니다.
if isaac_sim_ge_4_5_version:
    extensions.enable_extension("isaacsim.ros2.bridge")
else:
    extensions.enable_extension("omni.isaac.ros2_bridge")

# ─── Physics Inspector 확장 기능 활성화 ──────────────────────────────────────
# GUI에서 관절 상태를 수동으로 조작할 수 있는 Physics Inspector 패널을 활성화합니다.
# 수동 모드(manual mode) 전환 시 이 패널로 로봇을 직접 움직일 수 있습니다.
extensions.enable_extension("omni.physx.supportui")

# stage_units_in_meters=1.0: 씬의 단위를 미터로 설정합니다.
# URDF는 기본적으로 미터 단위이므로 1.0이 올바른 설정입니다.
simulation_context = SimulationContext(stage_units_in_meters=1.0)

# ─── 경로 설정 ────────────────────────────────────────────────────────────────
# m0609 로봇 URDF 경로.
# m0609_isaac_sim.urdf는 mesh 경로가 절대 경로로 수정된 전용 파일입니다.
# (Isaac Sim은 package:// URI를 해석하지 못해 표준 ROS URDF를 그대로 쓸 수 없습니다.)
ROBOT_URDF_PATH = "/home/ssu/dev_ws/issac_sim/src/m0609_urdf/urdf/m0609_isaac_sim.urdf"
MESH_OLD_PATH   = "/home/ssu/dev_ws/issac_sim/src/doosan-robot2/urdf/meshes/"
MESH_NEW_PATH   = "/home/ssu/dev_ws/issac_sim/src/m0609_urdf/urdf/meshes/"

# OnRobot RG2 그리퍼 URDF 경로
RG2_URDF_PATH   = "/home/ssu/dev_ws/issac_sim/src/onrobot_rg2/urdf/onrobot_rg2.urdf"
RG2_MESH_BASE   = "/home/ssu/dev_ws/issac_sim/src/onrobot_rg2/meshes"

# OmniGraph(Action Graph)를 생성할 USD 씬 경로
GRAPH_PATH      = "/ActionGraph"

# Isaac Sim Nucleus 서버에서 불러올 배경 환경 USD 파일 (Simple Room)
BACKGROUND_USD  = "/Isaac/Environments/Simple_Room/simple_room.usd"

# ─── 배경 환경 로드 ───────────────────────────────────────────────────────────
# Nucleus 서버의 루트 경로를 가져옵니다. 로컬 캐시 또는 원격 서버일 수 있습니다.
assets_root_path = nucleus.get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Isaac Sim assets 경로를 찾을 수 없습니다.")
    simulation_app.close()
    sys.exit()

# 뷰포트 카메라를 비스듬한 각도에서 로봇을 바라보도록 배치합니다.
# eye: 카메라 위치(XYZ), target: 바라보는 지점(XYZ)
viewports.set_camera_view(
    eye=np.array([1.5, 1.5, 1.0]),
    target=np.array([0.0, 0.0, 0.5])
)

# 배경 USD를 씬의 "/background" 경로에 추가합니다.
stage.add_reference_to_stage(assets_root_path + BACKGROUND_USD, "/background")
simulation_app.update()

# ─── m0609 + RG2 URDF 통합 ───────────────────────────────────────────────────
# Isaac Sim 은 package:// URI 를 해석할 수 없으므로 xacro 를 사용하지 않고
# 절대 경로 mesh 를 가진 m0609_isaac_sim.urdf 를 베이스로 직접 합칩니다.

def combine_robot_with_rg2(robot_urdf, mesh_old, mesh_new,
                            rg2_urdf, rg2_mesh_base,
                            attach_link="link_6", prefix="rg2_"):
    """
    m0609 URDF 와 RG2 URDF 를 하나로 합친 URDF 문자열을 반환합니다.

    Isaac Sim은 단일 URDF 파일만 임포트할 수 있으므로,
    로봇 암(m0609)과 그리퍼(RG2)를 하나의 URDF로 합쳐야 합니다.

    처리 순서:
      1. m0609 URDF 로드 후 mesh 경로를 절대 경로로 교체
      2. RG2 URDF 로드
      3. RG2의 링크(시각/충돌 형상)를 m0609 URDF에 추가
         - 'world' 링크는 제외 (m0609의 world 링크와 충돌 방지)
         - 링크 이름 앞에 prefix('rg2_')를 붙여 이름 충돌 방지
         - mesh 경로를 절대 경로로 교체
      4. RG2의 조인트(관절)를 m0609 URDF에 추가
         - 조인트 이름 앞에 prefix 추가
         - RG2의 root 조인트 parent를 'world' → attach_link(link_6)로 변경
           (그리퍼가 로봇 암 6번 링크 플랜지에 부착됨)
         - mimic 조인트 참조에도 prefix 추가

    Args:
        robot_urdf    : m0609 URDF 파일 경로
        mesh_old      : m0609 URDF 내 교체 전 mesh 경로 (구 경로)
        mesh_new      : m0609 URDF 내 교체 후 mesh 경로 (신 경로)
        rg2_urdf      : RG2 URDF 파일 경로
        rg2_mesh_base : RG2 mesh 절대 경로 베이스 디렉토리
        attach_link   : 그리퍼를 연결할 로봇 암의 링크 이름 (기본: link_6)
        prefix        : RG2 링크·조인트 이름에 붙일 접두사 (기본: rg2_)

    Returns:
        str: 합쳐진 URDF 문자열 (XML)
    """
    # 1. m0609 URDF 로드 + mesh 경로 수정
    # m0609_isaac_sim.urdf의 mesh 경로가 구 경로(doosan-robot2)를 참조하는 경우
    # 신 경로(m0609_urdf)로 교체합니다.
    with open(robot_urdf, "r") as f:
        robot_content = f.read().replace(mesh_old, mesh_new)
    robot_root = ET.fromstring(robot_content)

    # 2. RG2 URDF 로드
    rg2_root = ET.parse(rg2_urdf).getroot()

    # 3. RG2 링크 추가 (world 링크 제외)
    for link in rg2_root.findall("link"):
        name = link.get("name", "")
        if name == "world":
            # RG2의 world 링크는 m0609의 world 링크와 중복되므로 건너뜁니다.
            continue
        # 링크 이름에 prefix를 붙여 m0609 링크 이름과 충돌을 방지합니다.
        link.set("name", prefix + name)
        # RG2 mesh 경로를 상대 경로에서 절대 경로로 변환합니다.
        for mesh in link.findall(".//mesh"):
            fn = mesh.get("filename", "")
            # "../meshes/visual/xxx.stl" → "/abs/path/visual/xxx.stl"
            if fn.startswith("../meshes/"):
                rel = fn[len("../meshes/"):]   # e.g. "visual/quick_changer.stl"
                mesh.set("filename", f"{rg2_mesh_base}/{rel}")
        robot_root.append(link)

    # 4. RG2 조인트 추가
    for joint in rg2_root.findall("joint"):
        j_name = joint.get("name", "")
        parent_el = joint.find("parent")
        child_el  = joint.find("child")
        if parent_el is None or child_el is None:
            continue

        # 조인트 이름에도 prefix를 붙입니다.
        joint.set("name", prefix + j_name)
        # child 링크 이름에도 prefix를 붙입니다.
        child_el.set("link", prefix + child_el.get("link", ""))

        parent_link = parent_el.get("link", "")
        if parent_link == "world":
            # RG2의 루트 조인트(quick_changer_joint)는 원래 parent가 'world'입니다.
            # 이를 로봇 암의 마지막 링크(link_6)로 변경하여 그리퍼를 플랜지에 부착합니다.
            parent_el.set("link", attach_link)
            # 부착 위치 오프셋: link_6 기준 (0,0,0)으로 설정합니다.
            # 필요하다면 이 값을 조정하여 그리퍼 부착 위치를 변경할 수 있습니다.
            origin = joint.find("origin")
            if origin is not None:
                origin.set("xyz", "0 0 0")
                origin.set("rpy", "0 0 0")
            else:
                joint.insert(0, ET.Element("origin", xyz="0 0 0", rpy="0 0 0"))
        else:
            # RG2 내부 조인트의 parent 링크 이름에도 prefix를 붙입니다.
            parent_el.set("link", prefix + parent_link)

        # mimic 조인트: 한 조인트가 다른 조인트를 따라 움직이도록 설정합니다.
        # (RG2 그리퍼의 좌우 핑거가 연동되는 방식)
        # mimic이 참조하는 조인트 이름에도 prefix를 추가합니다.
        mimic = joint.find("mimic")
        if mimic is not None:
            mimic.set("joint", prefix + mimic.get("joint", ""))

        robot_root.append(joint)

    return '<?xml version="1.0"?>\n' + ET.tostring(robot_root, encoding="unicode")


# m0609와 RG2를 하나의 URDF 문자열로 합칩니다.
print("m0609 + RG2 URDF 통합 중...")
combined_urdf = combine_robot_with_rg2(
    robot_urdf=ROBOT_URDF_PATH,
    mesh_old=MESH_OLD_PATH,
    mesh_new=MESH_NEW_PATH,
    rg2_urdf=RG2_URDF_PATH,
    rg2_mesh_base=RG2_MESH_BASE,
    attach_link="link_6",   # 그리퍼 부착 위치: m0609의 6번 링크(플랜지)
    prefix="rg2_",          # RG2 링크·조인트 이름 앞에 붙을 접두사
)

# 합쳐진 URDF를 임시 파일로 저장합니다.
# Isaac Sim의 URDFParseAndImportFile 명령은 파일 경로를 입력받으므로
# 문자열을 파일로 저장해야 합니다.
# 스크립트 종료 시 이 임시 파일은 삭제됩니다.
tmp_urdf = tempfile.NamedTemporaryFile(
    mode="w", suffix=".urdf", delete=False, prefix="m0609_rg2_"
)
tmp_urdf.write(combined_urdf)
tmp_urdf.close()
fixed_urdf_path = tmp_urdf.name
print(f"통합 URDF 임시 파일 생성: {fixed_urdf_path}")

# ─── URDF Import 설정 ─────────────────────────────────────────────────────────
# ImportConfig는 URDF를 USD로 변환할 때의 옵션을 설정합니다.
import_config = _urdf.ImportConfig()

# fix_base=True: 로봇 베이스를 바닥에 고정합니다.
#   False로 하면 로봇 전체가 물리 시뮬레이션의 영향을 받아 넘어집니다.
import_config.fix_base              = True

# merge_fixed_joints=False: 고정 조인트를 유지합니다.
#   True로 하면 고정 조인트로 연결된 링크들을 하나로 합쳐 시뮬레이션 성능을 높이지만,
#   RG2 그리퍼처럼 별도로 제어해야 하는 링크가 사라질 수 있습니다.
import_config.merge_fixed_joints    = False

# import_inertia_tensor=True: URDF에 정의된 관성 텐서(inertia)를 사용합니다.
#   False면 Isaac Sim이 메시 형상으로부터 자동 계산합니다.
import_config.import_inertia_tensor = True

# distance_scale=1.0: URDF의 거리 단위가 미터일 때 1.0으로 설정합니다.
import_config.distance_scale        = 1.0

# make_default_prim=True: 임포트된 로봇을 씬의 기본 prim으로 설정합니다.
import_config.make_default_prim     = True

# create_physics_scene=True: 물리 씬을 자동으로 생성합니다.
import_config.create_physics_scene  = True

# default_drive_type: 조인트 구동 방식을 위치 제어(Position Drive)로 설정합니다.
# 위치 제어: 목표 각도를 입력하면 PD 제어기가 해당 위치로 이동시킵니다.
# (속도 제어나 토크 제어도 가능하지만 MoveIt2 연동에는 위치 제어가 일반적입니다.)
try:
    import_config.default_drive_type = _urdf.DriveType.DRIVE_POSITION
except AttributeError:
    try:
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
    except AttributeError:
        import_config.default_drive_type = 1  # 상수값 폴백

# 위치 제어 PD 게인 설정
# default_drive_strength: 스프링 강성(P 게인). 값이 클수록 목표 위치를 강하게 추종합니다.
# default_position_drive_damping: 댐핑(D 게인). 값이 클수록 진동을 억제합니다.
import_config.default_drive_strength         = 1e7
import_config.default_position_drive_damping = 1e5

# ─── URDF 임포트 실행 ─────────────────────────────────────────────────────────
# URDFParseAndImportFile 명령이 URDF 파일을 파싱하고 USD 씬에 로봇 prim을 생성합니다.
# 성공 시 status=True, robot_prim_path는 생성된 최상위 prim의 경로(예: "/m0609")
print("URDF 임포트 중...")
status, robot_prim_path = omni.kit.commands.execute(
    "URDFParseAndImportFile",
    urdf_path=fixed_urdf_path,
    import_config=import_config,
    dest_path="",
)

if not status:
    carb.log_error(f"URDF 임포트 실패: {fixed_urdf_path}")
    os.unlink(fixed_urdf_path)
    simulation_app.close()
    sys.exit()

print(f"URDF 임포트 완료 → 반환된 prim 경로: {robot_prim_path}")

# 복합 URDF(arm+gripper)의 물리 구조가 완전히 초기화될 때까지 여러 프레임을 업데이트합니다.
# 프레임 수가 부족하면 ArticulationRoot 탐색이 실패할 수 있습니다.
for _ in range(10):
    simulation_app.update()

# ─── ArticulationRoot prim 경로 자동 탐색 ────────────────────────────────────
# ArticulationRoot: Isaac Sim에서 관절 제어의 기준이 되는 prim입니다.
# URDF 임포트 후 반환되는 robot_prim_path(예: /m0609)가 ArticulationRoot가 아닐 수 있습니다.
# 실제 ArticulationRoot는 /m0609/root_joint처럼 한 단계 아래에 있을 수 있으므로
# 씬 전체를 순회하여 PhysicsArticulationRootAPI가 적용된 prim을 찾습니다.
def find_articulation_root(search_stage):
    """씬 전체를 순회하여 PhysicsArticulationRootAPI가 적용된 prim 경로를 반환합니다."""
    for prim in search_stage.Traverse():
        if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
            return str(prim.GetPath())
    return None

current_stage     = stage.get_current_stage()
articulation_path = find_articulation_root(current_stage)

if articulation_path and articulation_path != robot_prim_path:
    # 자동 탐색 성공: 찾은 ArticulationRoot 경로로 업데이트합니다.
    print(f"ArticulationRoot 발견: {articulation_path}")
    robot_prim_path = articulation_path
else:
    # 폴백 1: <base_prim>/root_joint 경로를 직접 확인합니다.
    # Isaac Sim URDF importer는 루트 조인트를 기본적으로 root_joint로 명명합니다.
    fallback = robot_prim_path.rstrip("/") + "/root_joint"
    if current_stage.GetPrimAtPath(fallback).IsValid():
        robot_prim_path = fallback
        print(f"[폴백] ArticulationRoot: {robot_prim_path}")
    else:
        print(f"[경고] ArticulationRoot를 찾지 못함 → 경로 유지: {robot_prim_path}")

# robot_body_path: PublishJointState 노드가 관절 상태를 읽어올 prim 경로
# ArticulationRoot와 동일한 경로를 사용합니다. (예: /m0609/root_joint)
robot_body_path = robot_prim_path

# ─── ROS_DOMAIN_ID 확인 ───────────────────────────────────────────────────────
# ROS2 DDS 네트워크 격리를 위한 도메인 ID입니다.
# MoveIt2 실행 환경과 동일한 DOMAIN_ID를 사용해야 토픽이 연결됩니다.
# 환경 변수가 없으면 기본값 0을 사용합니다.
try:
    ros_domain_id = int(os.environ.get("ROS_DOMAIN_ID", 0))
except ValueError:
    ros_domain_id = 0
print(f"ROS_DOMAIN_ID: {ros_domain_id}")

# ─── OmniGraph (Action Graph) 생성 ───────────────────────────────────────────
# OmniGraph는 Isaac Sim의 노드 기반 데이터플로우 시스템입니다.
# 여기서는 매 시뮬레이션 스텝마다 ROS2 토픽을 발행·구독하는 그래프를 구성합니다.
#
# 그래프 구조:
#   [Publish 브랜치] - 항상 실행
#     OnImpulseEventPub → PublishJointState  (관절 상태를 /isaac_joint_states로 발행)
#     OnImpulseEventPub → PublishClock       (시뮬레이션 시간을 /clock으로 발행)
#
#   [Control 브랜치] - 수동 모드에서는 정지
#     OnImpulseEventCtrl → SubscribeJointState    (/isaac_joint_commands 토픽 구독)
#     OnImpulseEventCtrl → ArticulationController (수신한 명령으로 관절 구동)
#
# 두 브랜치를 분리한 이유:
#   수동 모드에서는 Physical Inspector로 관절을 수동 조작하는데,
#   ArticulationController가 동시에 실행되면 명령을 덮어써 버립니다.
#   Control 브랜치를 멈추면 수동 조작값이 그대로 유지됩니다.

# Isaac Sim 버전에 따라 노드 타입 이름이 다릅니다.
if isaac_sim_ge_4_5_version:
    NODES = {
        "impulse":  "omni.graph.action.OnImpulseEvent",        # 매 프레임 트리거 노드
        "sim_time": "isaacsim.core.nodes.IsaacReadSimulationTime",  # 시뮬레이션 시간 읽기
        "context":  "isaacsim.ros2.bridge.ROS2Context",        # ROS2 DDS 컨텍스트
        "pub_js":   "isaacsim.ros2.bridge.ROS2PublishJointState",   # 관절 상태 발행
        "sub_js":   "isaacsim.ros2.bridge.ROS2SubscribeJointState", # 관절 명령 구독
        "artic":    "isaacsim.core.nodes.IsaacArticulationController",  # 관절 구동 제어기
        "pub_clk":  "isaacsim.ros2.bridge.ROS2PublishClock",   # 시뮬레이션 클럭 발행
    }
else:
    NODES = {
        "impulse":  "omni.graph.action.OnImpulseEvent",
        "sim_time": "omni.isaac.core_nodes.IsaacReadSimulationTime",
        "context":  "omni.isaac.ros2_bridge.ROS2Context",
        "pub_js":   "omni.isaac.ros2_bridge.ROS2PublishJointState",
        "sub_js":   "omni.isaac.ros2_bridge.ROS2SubscribeJointState",
        "artic":    "omni.isaac.core_nodes.IsaacArticulationController",
        "pub_clk":  "omni.isaac.ros2_bridge.ROS2PublishClock",
    }

# OmniGraph 노드 속성값 설정 목록
og_set_values = [
    ("Context.inputs:domain_id",                ros_domain_id),   # ROS2 도메인 ID
    ("ArticulationController.inputs:robotPath", robot_prim_path), # 제어할 로봇 prim 경로
    ("PublishJointState.inputs:topicName",      "isaac_joint_states"),   # 발행 토픽 이름
    ("SubscribeJointState.inputs:topicName",    "isaac_joint_commands"), # 구독 토픽 이름
]
# 레거시 Isaac Sim에서는 ArticulationController에 usePath 플래그를 명시해야 합니다.
if is_legacy_isaacsim:
    og_set_values.insert(1, ("ArticulationController.inputs:usePath", True))

try:
    og.Controller.edit(
        {"graph_path": GRAPH_PATH, "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                # Publish 브랜치 — 항상 실행 (joint state / clock 발행)
                ("OnImpulseEventPub",      NODES["impulse"]),
                # Control 브랜치 — 수동 모드에서 정지 (ArticulationController 비활성화)
                ("OnImpulseEventCtrl",     NODES["impulse"]),
                ("ReadSimTime",            NODES["sim_time"]),
                ("Context",                NODES["context"]),
                ("PublishJointState",      NODES["pub_js"]),
                ("SubscribeJointState",    NODES["sub_js"]),
                ("ArticulationController", NODES["artic"]),
                ("PublishClock",           NODES["pub_clk"]),
            ],
            og.Controller.Keys.CONNECT: [
                # Publish 브랜치: 관절 상태와 클럭을 ROS2로 발행
                ("OnImpulseEventPub.outputs:execOut",  "PublishJointState.inputs:execIn"),
                ("OnImpulseEventPub.outputs:execOut",  "PublishClock.inputs:execIn"),
                # Control 브랜치: MoveIt2 명령을 수신하여 관절 구동
                ("OnImpulseEventCtrl.outputs:execOut", "SubscribeJointState.inputs:execIn"),
                ("OnImpulseEventCtrl.outputs:execOut", "ArticulationController.inputs:execIn"),
                # ROS2 컨텍스트를 모든 ROS2 노드에 공유
                ("Context.outputs:context",            "PublishJointState.inputs:context"),
                ("Context.outputs:context",            "SubscribeJointState.inputs:context"),
                ("Context.outputs:context",            "PublishClock.inputs:context"),
                # 시뮬레이션 타임스탬프를 발행 노드에 연결 (ROS2 메시지 헤더용)
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                # SubscribeJointState에서 수신한 명령을 ArticulationController로 전달
                ("SubscribeJointState.outputs:jointNames",      "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand",   "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: og_set_values,
        },
    )
    print("Action Graph 생성 완료")
except Exception as e:
    carb.log_error(f"Action Graph 생성 실패: {e}")

# PublishJointState 노드에 관절 상태를 읽어올 로봇 prim을 연결합니다.
# 이 설정이 없으면 어떤 로봇의 관절을 발행할지 알 수 없습니다.
if isaac_sim_ge_4_5_version:
    set_targets(
        prim=stage.get_current_stage().GetPrimAtPath(f"{GRAPH_PATH}/PublishJointState"),
        attribute="inputs:targetPrim",
        target_prim_paths=[robot_body_path],
    )
else:
    from omni.isaac.core_nodes.scripts.utils import set_target_prims
    set_target_prims(
        primPath=f"{GRAPH_PATH}/PublishJointState",
        targetPrimPaths=[robot_body_path],
    )

simulation_app.update()
simulation_app.update()

# ─── 물리 초기화 및 시뮬레이션 시작 ─────────────────────────────────────────
# initialize_physics(): 물리 씬(중력, 충돌 등)을 초기화합니다.
# play(): 시뮬레이션 재생을 시작합니다. 이후 step()으로 매 프레임을 진행합니다.
simulation_context.initialize_physics()
simulation_context.play()
simulation_app.update()

print("=" * 60)
print("Isaac Sim m0609 + RG2 시뮬레이션 시작")
print(f"  ArticulationRoot : {robot_prim_path}")
print(f"  PublishJointState target : {robot_body_path}")
print(f"  발행 토픽        : /isaac_joint_states")
print(f"  구독 토픽        : /isaac_joint_commands")
print(f"  clock 토픽       : /clock")
print("=" * 60)
print("이제 터미널 B에서 MoveIt2 launch 파일을 실행하세요:")
print("  ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py")

# ─── 수동 모드 플래그 설정 ────────────────────────────────────────────────────
# 수동 모드: 파일 플래그(/tmp/isaac_manual_mode)의 존재 여부로 제어합니다.
# ROS2 노드를 별도로 실행하지 않고 프로세스 간 통신하는 간단한 방법입니다.
#
# 수동 모드 전환 방법:
#   활성화: touch /tmp/isaac_manual_mode
#   비활성화: rm /tmp/isaac_manual_mode
#
# 수동 모드가 활성화되면 ArticulationController가 멈추어
# Physics Inspector 패널로 관절을 직접 조작할 수 있습니다.
_MANUAL_MODE_FLAG = "/tmp/isaac_manual_mode"

# 시작 시 혹시 남아있을 수 있는 플래그 파일을 제거합니다.
if os.path.exists(_MANUAL_MODE_FLAG):
    os.remove(_MANUAL_MODE_FLAG)

_last_manual_mode = False  # 모드 변경 감지를 위한 이전 상태 저장

# ─── 메인 루프 ────────────────────────────────────────────────────────────────
# simulation_app.is_running(): Isaac Sim GUI 창이 열려 있는 동안 True를 반환합니다.
# 창을 닫거나 Ctrl+C를 누르면 False가 되어 루프가 종료됩니다.
while simulation_app.is_running():
    # 물리 시뮬레이션을 한 스텝 진행하고 화면을 렌더링합니다.
    simulation_context.step(render=True)

    # 수동 모드 플래그 파일 존재 여부를 확인합니다.
    _current_manual = os.path.exists(_MANUAL_MODE_FLAG)

    # Publish 브랜치: 수동 모드 여부와 관계없이 항상 실행합니다.
    # MoveIt2가 현재 관절 상태를 지속적으로 수신해야 하기 때문입니다.
    og.Controller.set(
        og.Controller.attribute(f"{GRAPH_PATH}/OnImpulseEventPub.state:enableImpulse"),
        True,
    )

    # Control 브랜치: 수동 모드가 아닐 때만 실행합니다.
    # → ArticulationController가 드라이브 타겟을 덮어쓰지 않으므로
    #   Physical Inspector로 설정한 위치가 그대로 유지됩니다.
    if not _current_manual:
        og.Controller.set(
            og.Controller.attribute(f"{GRAPH_PATH}/OnImpulseEventCtrl.state:enableImpulse"),
            True,
        )

    # 수동 모드가 변경된 경우에만 로그를 출력합니다.
    if _current_manual != _last_manual_mode:
        _last_manual_mode = _current_manual
        if _current_manual:
            print("수동 모드: ArticulationController 정지 → Physical Inspector로 조작 가능")
        else:
            print("MoveIt2 모드: ArticulationController 재개")

# ─── 종료 처리 ────────────────────────────────────────────────────────────────
simulation_context.stop()          # 물리 시뮬레이션 정지
os.unlink(fixed_urdf_path)         # 임시 URDF 파일 삭제
simulation_app.close()             # Isaac Sim GUI 창 닫기
