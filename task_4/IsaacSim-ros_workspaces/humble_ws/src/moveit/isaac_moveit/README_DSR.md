# Doosan m0609 + OnRobot RG2 — Isaac Sim / MoveIt2 연동 가이드

Doosan m0609 암과 OnRobot RG2 그리퍼를 Isaac Sim 시뮬레이터, 실물 로봇, 가상 에뮬레이터와 MoveIt2로 연동하는 방법을 설명합니다.

---

## 목차

1. [시스템 구조](#시스템-구조)
2. [사전 준비](#사전-준비)
3. [컴퓨터별 경로 설정](#컴퓨터별-경로-설정)
4. [실행 방법](#실행-방법)
   - [Isaac Sim 모드](#isaac-sim-모드-기본)
   - [실물 로봇 모드](#실물-로봇-모드)
   - [Doosan 가상 에뮬레이터 모드](#doosan-가상-에뮬레이터-모드)
   - [Mock 모드 (하드웨어 없이 MoveIt2 단독)](#mock-모드)
5. [수동 조작 모드 (Physical Inspector 동기화)](#수동-조작-모드)
6. [주요 파일 목록](#주요-파일-목록)
7. [트러블슈팅](#트러블슈팅)

---

## 시스템 구조

```
┌─────────────────────────────────────────────────────────────────┐
│  Isaac Sim (isaac_moveit_m0609_rg2.py)                          │
│  ┌─────────────────────────────────────────────────────────┐    │
│  │ OmniGraph Action Graph                                  │    │
│  │  PublishJointState  →→  /isaac_joint_states            │    │
│  │  SubscribeJointState ←←  /isaac_joint_commands         │    │
│  │  PublishClock       →→  /clock                         │    │
│  └─────────────────────────────────────────────────────────┘    │
└───────────────────────────────────────────────────────────┬─────┘
                                                            │ ROS2
┌───────────────────────────────────────────────────────────┼─────┐
│  MoveIt2 (isaac_moveit_dsr_m0609_rg2.launch.py)           │     │
│                                                           │     │
│  TopicBasedSystem (ros2_control hardware)                 │     │
│    /isaac_joint_states ←────────────────────────────────←─┘     │
│    /isaac_joint_commands →──────────────────────────────→─┐     │
│                                                           │     │
│  joint_state_broadcaster → /joint_states                  │     │
│  dsr_moveit_controller   ↕ FollowJointTrajectory          │     │
│  rg2_gripper_controller  ↕ FollowJointTrajectory          │     │
│                                                           │     │
│  move_group (MoveIt2)                                     │     │
│  RViz2 (interactive marker, Plan & Execute)               │     │
│                                                           │     │
│  joint_state_passthrough (수동 모드 시 echo)               │     │
└───────────────────────────────────────────────────────────┴─────┘
```

---

## 사전 준비

### 필요한 워크스페이스 및 패키지

| 항목 | 기본 경로 | 비고 |
|---|---|---|
| Doosan ROS2 워크스페이스 | `~/dev_ws/issac_sim/src/doosan_ros2` | `dsr_description2`, `dsr_moveit_config_m0609` 포함 |
| Isaac MoveIt 워크스페이스 | `~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws` | 이 패키지 (`isaac_moveit`) |
| m0609 URDF | `~/dev_ws/issac_sim/src/m0609_urdf` | Isaac Sim 전용 절대경로 mesh URDF |
| OnRobot RG2 URDF | `~/dev_ws/issac_sim/src/onrobot_rg2` | 그리퍼 mesh 및 URDF |
| Isaac Sim 설치 | `~/dev_ws/issac_sim/isaacsim` | `python.sh` 포함 |

### 빌드

```bash
# 1. Doosan 워크스페이스 빌드 (최초 1회)
cd ~/dev_ws/issac_sim/src/doosan_ros2
colcon build

# 2. Isaac MoveIt 워크스페이스 빌드
cd ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws
colcon build --packages-select isaac_moveit
```

---

## 컴퓨터별 경로 설정

> **새 컴퓨터에서 처음 설정할 때 반드시 아래 파일들의 경로를 수정하세요.**

### 1. Isaac Sim 실행 스크립트 — `isaac_moveit_m0609_rg2.py`

경로: `~/dev_ws/issac_sim/src/isaac_moveit_m0609_rg2.py`

```python
# ── 수정 필요 항목 ──────────────────────────────────────────────
# m0609 URDF (절대경로 mesh 버전)
ROBOT_URDF_PATH = "/home/<사용자명>/dev_ws/issac_sim/src/m0609_urdf/urdf/m0609_isaac_sim.urdf"

# URDF 내부 mesh 경로 교체 (원본 → 실제 위치)
MESH_OLD_PATH   = "/home/<사용자명>/dev_ws/issac_sim/src/doosan-robot2/urdf/meshes/"
MESH_NEW_PATH   = "/home/<사용자명>/dev_ws/issac_sim/src/m0609_urdf/urdf/meshes/"

# RG2 그리퍼 URDF 및 mesh 경로
RG2_URDF_PATH   = "/home/<사용자명>/dev_ws/issac_sim/src/onrobot_rg2/urdf/onrobot_rg2.urdf"
RG2_MESH_BASE   = "/home/<사용자명>/dev_ws/issac_sim/src/onrobot_rg2/meshes"
```

Isaac Sim Python 실행 경로도 확인하세요:

```bash
# 설치 경로 탐색
find ~ -name "python.sh" 2>/dev/null | grep isaac
# 예시 결과: /home/<사용자명>/dev_ws/issac_sim/isaacsim/_build/linux-x86_64/release/python.sh
```

### 2. RG2 xacro mesh 경로 — `config/onrobot_rg2_isaac.xacro`

경로: `config/onrobot_rg2_isaac.xacro`

```xml
<!-- 6번째 줄 근처, 절대경로 수정 -->
<xacro:property name="rg2_mesh"
    value="file:///home/<사용자명>/dev_ws/issac_sim/src/onrobot_rg2/meshes"/>
```

> `file://` 접두사가 반드시 있어야 RViz2에서 mesh가 표시됩니다.

### 3. 실물 로봇 IP — launch 실행 시 인수

별도 파일 수정 없이 launch 명령어에서 `host` 인수로 지정합니다.

```bash
# 예: 실물 로봇 IP가 192.168.1.100인 경우
ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
  mode:=real host:=192.168.1.100
```

| 인수 | 기본값 | 설명 |
|---|---|---|
| `host` | `127.0.0.1` | 로봇 제어기 IP |
| `port` | `12345` | 로봇 제어기 포트 |
| `rt_host` | `192.168.137.50` | RT 제어 IP (실시간 제어 시) |

---

## 실행 방법

### Isaac Sim 모드 (기본)

Isaac Sim에서 m0609 + RG2를 시뮬레이션하고 MoveIt2로 제어합니다.

**터미널 A — Isaac Sim 실행**

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
source ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

~/dev_ws/issac_sim/isaacsim/_build/linux-x86_64/release/python.sh \
  ~/dev_ws/issac_sim/src/isaac_moveit_m0609_rg2.py
```

Isaac Sim 창이 열리고 아래 메시지가 출력되면 준비 완료:

```
============================================================
Isaac Sim m0609 + RG2 시뮬레이션 시작
  ArticulationRoot : /m0609/root_joint
  발행 토픽        : /isaac_joint_states
  구독 토픽        : /isaac_joint_commands
  clock 토픽       : /clock
============================================================
```

**터미널 B — MoveIt2 실행**

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
source ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py
```

컨트롤러 실행 순서 (자동):
```
ros2_control_node 시작
  └─ (3초 대기) joint_state_broadcaster 활성화
       └─ dsr_moveit_controller 활성화
            └─ rg2_gripper_controller 활성화
                 └─ move_group + RViz2 + joint_state_passthrough 시작
```

**연동 확인**

```bash
# Joint states 수신 확인
ros2 topic echo /isaac_joint_states

# 컨트롤러 상태 확인
ros2 control list_controllers
```

---

### 실물 로봇 모드

**터미널 A — MoveIt2 + 로봇 연결**

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
source ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
  mode:=real \
  host:=192.168.1.100
```

컨트롤러 실행 순서 (자동):
```
ros2_control_node + run_emulator 시작
  └─ (5초 대기) joint_state_broadcaster 활성화
       └─ dsr_controller2 활성화
            └─ dsr_moveit_controller 활성화
                 └─ rg2_gripper_controller 활성화
                      └─ move_group + RViz2 시작
```

> **주의**: 실물 로봇 연결 시 로봇이 이미 활성화(servo-on) 상태여야 합니다.  
> RG2 그리퍼는 `mock_components`로 동작하며 MoveIt2 충돌 회피에만 반영됩니다.  
> 실제 그리퍼 구동은 OnRobot 전용 드라이버를 별도로 사용하세요.

---

### Doosan 가상 에뮬레이터 모드

실물 로봇 없이 Doosan 내장 소프트웨어 에뮬레이터로 테스트합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
source ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
  mode:=virtual
```

실행 순서는 실물 모드와 동일합니다.

---

### Mock 모드

하드웨어 없이 MoveIt2와 RViz2만 실행해 플래닝을 테스트합니다.

```bash
source /opt/ros/humble/setup.bash
source ~/dev_ws/issac_sim/src/doosan_ros2/install/setup.bash
source ~/dev_ws/issac_sim/src/IsaacSim-ros_workspaces/humble_ws/install/setup.bash

ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
  mode:=mock
```

---

## 수동 조작 모드

Isaac Sim의 **Physics Inspector**로 joint를 직접 움직이면 MoveIt2 RViz2에도 실시간 반영됩니다.

### 활성화 방법

**터미널 C — 수동 모드 전환**

```bash
# 수동 모드 진입 (Physical Inspector 조작 가능)
ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: true"
```

내부 동작:
1. `joint_state_passthrough` 노드가 `dsr_moveit_controller`, `rg2_gripper_controller` 비활성화
2. Isaac Sim 스크립트가 모든 joint의 drive stiffness를 0으로 설정
3. Physics Inspector로 joint 자유 조작 가능
4. 조작된 위치가 `/isaac_joint_states` → `/joint_states` 경로로 MoveIt2에 반영

**MoveIt2 제어 복귀**

```bash
# MoveIt2 모드 복귀 (Plan & Execute 사용 가능)
ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: false"
```

> Isaac Sim 터미널에서 아래 메시지가 출력되면 정상 전환된 것입니다:
> ```
> 수동 모드: joint drive 비활성화 → Physical Inspector로 조작 가능
>   → N개 joint drive 설정 (stiffness=0e+00, damping=1e+03)
> ```

---

## 주요 파일 목록

```
# ── Isaac Sim 실행 스크립트 ──────────────────────────────────────
~/dev_ws/issac_sim/src/
└── isaac_moveit_m0609_rg2.py          Isaac Sim 시뮬레이션 스크립트
                                        (URDF 통합, OmniGraph 설정)

# ── isaac_moveit ROS2 패키지 ─────────────────────────────────────
isaac_moveit/
├── config/
│   ├── dsr_m0609_rg2_isaac.urdf.xacro   Isaac Sim 모드 URDF (TopicBasedSystem)
│   ├── dsr_m0609_rg2_real.urdf.xacro    실물/virtual 모드 URDF (DRHWInterface)
│   ├── dsr_m0609_rg2_ros2_control.xacro ros2_control 하드웨어 인터페이스 xacro
│   ├── dsr_m0609_rg2.srdf               MoveIt2 Semantic Description
│   ├── onrobot_rg2_isaac.xacro          RG2 그리퍼 xacro macro
│   ├── m0609_rg2_controllers.yaml       Isaac/mock 모드 컨트롤러 설정
│   ├── m0609_rg2_real_controllers.yaml  실물/virtual 모드 컨트롤러 설정
│   └── m0609_rg2_moveit_controllers.yaml MoveIt2 컨트롤러 매핑
├── launch/
│   └── isaac_moveit_dsr_m0609_rg2.launch.py  통합 launch 파일 (mode 인수로 분기)
├── scripts/
│   └── joint_state_passthrough.py       수동 모드 동기화 노드
└── README_DSR.md                        이 파일
```

---

## 트러블슈팅

### Isaac Sim에서 로봇 모델이 나타나지 않음

`isaac_moveit_m0609_rg2.py`의 경로 설정을 확인하세요.

```python
# 파일이 실제로 존재하는지 확인
ls $ROBOT_URDF_PATH
ls $RG2_URDF_PATH
```

mesh 파일 경로가 URDF 내부와 다른 경우 `MESH_OLD_PATH → MESH_NEW_PATH` 치환이 작동하지 않으면:

```bash
# URDF 내부 실제 mesh 경로 확인
grep "filename" $ROBOT_URDF_PATH | head -5
```

### RViz2에서 RG2 mesh가 표시되지 않음

`config/onrobot_rg2_isaac.xacro`의 `rg2_mesh` 경로에 `file://` 접두사가 있는지,  
절대경로가 현재 컴퓨터의 실제 경로와 일치하는지 확인하세요.

```bash
# mesh 파일 존재 확인
ls /home/<사용자명>/dev_ws/issac_sim/src/onrobot_rg2/meshes/visual/
```

### joint drive 비활성화 메시지에서 N=0

수동 모드 전환 시 `→ 0개 joint drive 설정` 이 출력되면 prim 경로가 맞지 않는 것입니다.  
Isaac Sim 터미널에서 `_robot_prim_root` 값을 확인하세요.

```python
# isaac_moveit_m0609_rg2.py 에서 임시 출력 추가
print(f"robot_prim_root: {_robot_prim_root}")
```

스테이지에서 로봇 prim 경로가 `/m0609`이 아닌 경우 스크립트 상단의 URDF import 결과를 확인하세요.

### `dsr_controller2` spawner 타임아웃 (실물/virtual 모드)

`run_emulator` 노드가 준비되기 전에 컨트롤러가 활성화를 시도하면 발생합니다.  
`launch.py`에서 `jsb` 시작 전 대기 시간을 늘려보세요:

```python
# _real_mode 함수 내 TimerAction period 수정
TimerAction(period=8.0, actions=[jsb])  # 5.0 → 8.0
```

### `Semantic description is not specified for the same robot as the URDF`

URDF의 `<robot name="...">` 과 SRDF의 `<robot name="...">` 이 다른 경우입니다.  
두 파일 모두 `m0609_rg2` 인지 확인하세요.

```bash
# URDF robot name 확인
grep "robot name" $(ros2 pkg prefix isaac_moveit)/share/isaac_moveit/config/dsr_m0609_rg2_isaac.urdf.xacro

# SRDF robot name 확인
grep "robot name" $(ros2 pkg prefix isaac_moveit)/share/isaac_moveit/config/dsr_m0609_rg2.srdf
```

### MoveIt2 Plan이 자꾸 실패 (self-collision)

SRDF의 `disable_collisions` 목록이 불충분한 경우입니다.  
현재 상태에서 충돌 쌍을 확인하려면:

```bash
ros2 service call /check_state_validity moveit_msgs/srv/GetStateValidity \
  "{group_name: 'manipulator'}"
```

출력된 `contact_groups`의 링크 쌍을 `config/dsr_m0609_rg2.srdf`에 추가하세요.
