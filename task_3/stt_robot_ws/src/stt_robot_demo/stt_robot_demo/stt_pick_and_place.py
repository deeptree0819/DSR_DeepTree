#!/usr/bin/env python3
"""
stt_pick_and_place.py — 행동 실행 노드

/robot_command (VoiceCommand) 구독
  → 명령 큐 → 워커 스레드에서 MoveItPy + OnRobot 그리퍼 실행
  → /robot_status  (String)  퍼블리시
  → /tts_input     (String)  퍼블리시

지원 명령:
  CMD_HOME          — 홈 자세 복귀
  CMD_PICK          — 픽만 실행 (task_id 지원)
  CMD_PLACE         — 플레이스만 실행 (task_id 지원)
  CMD_PICKPLACE     — 픽 → 플레이스 연속 (task_id=0: 전체, 1~4: 특정 기어)
  CMD_STOP          — 큐 비우기 + 취소 플래그
  CMD_GRIPPER_OPEN  — 그리퍼 직접 열기
  CMD_GRIPPER_CLOSE — 그리퍼 직접 닫기
  CMD_JOG           — 현재 EE 위치 기준 방향 이동 (direction, offset_m)
"""

import concurrent.futures
import math
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import String

from geometry_msgs.msg import Pose, PoseStamped
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from stt_robot_interfaces.msg import VoiceCommand

from .onrobot import RG


# ── 로봇 상수 ────────────────────────────────────────────
GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK    = "link_6"

HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}

# ── 그리퍼 파라미터 ──────────────────────────────────────
GRIPPER_NAME          = "rg2"
TOOLCHARGER_IP        = "192.168.1.1"
TOOLCHARGER_PORT      = 502
GRIPPER_OPEN_WIDTH_MM  = 50.0   # mm
GRIPPER_CLOSE_WIDTH_MM = 20.0   # mm
GRIPPER_FORCE_N        = 20.0   # N

# ── 안전 작업 영역 (base_link 기준) ─────────────────────
SAFE_X_MIN = 0.0
SAFE_Y_MIN = -0.3
SAFE_Y_MAX =  0.3
SAFE_Z_MIN =  0.27

# ── 기어 작업 목록 (robot_controller.py의 GEAR_TASKS와 동일) ─
GEAR_TASKS = [
    {"pick":  {"pos": (0.398,  0.096, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)},
     "place": {"pos": (0.398, -0.206, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)}},
    {"pick":  {"pos": (0.392,  0.200, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)},
     "place": {"pos": (0.392, -0.101, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)}},
    {"pick":  {"pos": (0.486,  0.153, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)},
     "place": {"pos": (0.486, -0.149, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)}},
    {"pick":  {"pos": (0.427,  0.148, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)},
     "place": {"pos": (0.426, -0.153, 0.280), "ori": (0.0, 1.0, 0.0, 0.0)}},
]

APPROACH_OFFSET  = 0.05   # m
JOG_OFFSET       = 0.05   # m  — 방향 이동 한 스텝 거리
WIGGLE_Z         = 0.295
WIGGLE_YAW_DEG   = 5.0
WIGGLE_COUNT     = 3


# ════════════════════════════════════════════════════
#   유틸 함수
# ════════════════════════════════════════════════════
def _clamp(x: float, y: float, z: float, logger) -> tuple[float, float, float]:
    if x < SAFE_X_MIN:
        logger.warning(f"x 클램핑: {x:.3f} → {SAFE_X_MIN:.3f}")
        x = SAFE_X_MIN
    if y < SAFE_Y_MIN:
        logger.warning(f"y 클램핑: {y:.3f} → {SAFE_Y_MIN:.3f}")
        y = SAFE_Y_MIN
    elif y > SAFE_Y_MAX:
        logger.warning(f"y 클램핑: {y:.3f} → {SAFE_Y_MAX:.3f}")
        y = SAFE_Y_MAX
    if z < SAFE_Z_MIN:
        logger.warning(f"z 클램핑: {z:.3f} → {SAFE_Z_MIN:.3f}")
        z = SAFE_Z_MIN
    return x, y, z


def _rot_matrix_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """3x3 회전행렬 → 쿼터니언 (x, y, z, w)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return x, y, z, w


def _make_posestamped(pos: tuple, ori: tuple, z_override: float | None = None) -> PoseStamped:
    ps = PoseStamped()
    ps.header.frame_id    = BASE_FRAME
    ps.pose.position.x    = pos[0]
    ps.pose.position.y    = pos[1]
    ps.pose.position.z    = z_override if z_override is not None else pos[2]
    ps.pose.orientation.x = ori[0]
    ps.pose.orientation.y = ori[1]
    ps.pose.orientation.z = ori[2]
    ps.pose.orientation.w = ori[3]
    return ps


def _yaw_quat(yaw_rad: float) -> tuple:
    return (math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2))


def _quat_mul(q1: tuple, q2: tuple) -> tuple:
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


# ════════════════════════════════════════════════════
#   메인 노드
# ════════════════════════════════════════════════════
class SttPickAndPlaceNode(Node):

    def __init__(self):
        super().__init__('stt_pick_and_place')

        # ── ROS 파라미터 ──────────────────────────────────
        self.declare_parameter('vel_scale',  0.15)
        self.declare_parameter('approach_m', APPROACH_OFFSET)
        self.declare_parameter('gripper_ip', TOOLCHARGER_IP)
        self.declare_parameter('use_gripper', True)

        vel_scale    = self.get_parameter('vel_scale').value
        approach_m   = self.get_parameter('approach_m').value
        gripper_ip   = self.get_parameter('gripper_ip').value
        use_gripper  = self.get_parameter('use_gripper').value

        self._approach  = approach_m
        self._vel_scale = vel_scale

        # ── MoveIt 초기화 ─────────────────────────────────
        self._robot = MoveItPy(node_name='moveit_py')
        self._arm   = self._robot.get_planning_component(GROUP_NAME)
        self._robot_model = self._robot.get_robot_model()

        self._home_params = self._make_plan_params("ompl", "RRTConnect", vel_scale, 0.1)
        self._pilz_params = self._make_plan_params(
            "pilz_industrial_motion_planner", "PTP", vel_scale, 0.1
        )

        # MoveIt은 단일 전용 스레드에서만 안전하게 실행
        self._motion_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="motion"
        )
        self._cancel_event = threading.Event()

        # ── OnRobot 그리퍼 ────────────────────────────────
        self._use_gripper = use_gripper
        self._gripper = None
        if use_gripper:
            try:
                self._gripper = RG(GRIPPER_NAME, gripper_ip, TOOLCHARGER_PORT)
                time.sleep(0.5)
                self._set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N)
                self.get_logger().info(f"그리퍼 초기화 완료 (IP: {gripper_ip})")
            except Exception as e:
                self.get_logger().error(f"그리퍼 초기화 실패: {e} — 시뮬레이션 모드로 전환")
                self._gripper = None

        # ── 내부 상태 ─────────────────────────────────────
        self._holding    = False   # 현재 물체 파지 여부
        self._held_gear  = 0       # 파지 중인 기어 번호 (0=없음)
        self._robot_state = 'IDLE' # IDLE/HOMING/PICKING/HOLDING/PLACING/JOGGING/ERROR
        self._cmd_q: queue.Queue[VoiceCommand] = queue.Queue()

        # ── 구독 ──────────────────────────────────────────
        self.create_subscription(VoiceCommand, '/robot_command', self._cmd_cb, 10)

        # ── 퍼블리셔 ──────────────────────────────────────
        self._status_pub     = self.create_publisher(String, '/robot_status', 10)
        self._tts_pub        = self.create_publisher(String, '/tts_input',    10)
        self._state_pub      = self.create_publisher(String, '/robot_state',  10)

        # ── 상태 퍼블리시 타이머 (2 Hz) ──────────────────
        self.create_timer(0.5, self._publish_state)

        # ── 워커 스레드 시작 ──────────────────────────────
        threading.Thread(target=self._worker, daemon=True).start()

        self.get_logger().info(
            '행동 실행 노드 준비 완료\n'
            f'  vel_scale={vel_scale}  approach={approach_m}m\n'
            f'  그리퍼: {"활성 (" + gripper_ip + ")" if self._gripper else "시뮬레이션"}\n'
            f'  기어 작업 수: {len(GEAR_TASKS)}'
        )

    # ────────────────────────────────────────────
    #   MoveIt 플래닝 파라미터
    # ────────────────────────────────────────────
    def _make_plan_params(self, pipeline, planner_id, vel, acc):
        p = PlanRequestParameters(self._robot)
        p.planning_pipeline              = pipeline
        p.planner_id                     = planner_id
        p.max_velocity_scaling_factor    = vel
        p.max_acceleration_scaling_factor = acc
        p.planning_time                  = 3.0
        return p

    # ────────────────────────────────────────────
    #   명령 수신 콜백
    # ────────────────────────────────────────────
    def _cmd_cb(self, msg: VoiceCommand):
        log = self.get_logger()

        if msg.command == VoiceCommand.CMD_STOP:
            cleared = 0
            while not self._cmd_q.empty():
                try:
                    self._cmd_q.get_nowait()
                    cleared += 1
                except queue.Empty:
                    break
            self._cancel_event.set()
            log.info(f'[STOP] 명령 큐 {cleared}개 취소, 취소 플래그 설정')
            self._publish_tts('동작을 중지했습니다.')
            return

        if msg.command == VoiceCommand.CMD_UNKNOWN:
            log.warn(f'알 수 없는 명령 무시: "{msg.raw_text}"')
            return

        self._cmd_q.put(msg)
        task_info = f"기어{msg.task_id}" if msg.task_id > 0 else "전체"
        log.info(
            f'[큐 추가] cmd={msg.message} task={task_info} '
            f'wiggle={msg.use_wiggle} (큐={self._cmd_q.qsize()})'
        )

    # ────────────────────────────────────────────
    #   워커 스레드
    # ────────────────────────────────────────────
    def _worker(self):
        logger = get_logger('stt_pick_and_place.worker')

        while True:
            try:
                msg: VoiceCommand = self._cmd_q.get(timeout=1.0)
            except queue.Empty:
                continue

            self._cancel_event.clear()
            cmd = msg.command
            task_id    = msg.task_id
            use_wiggle = msg.use_wiggle

            logger.info(f'===== {msg.message} 실행 (기어={task_id or "전체"}) =====')
            ok = True
            silent_fail = False  # True면 결과 TTS 생략 (이미 이유를 말한 경우)

            if cmd == VoiceCommand.CMD_HOME:
                self._set_state('HOMING')
                self._publish_tts('홈 자세로 이동합니다.')
                ok = self._move_home()
                self._set_state('HOLDING' if self._holding else 'IDLE')

            elif cmd == VoiceCommand.CMD_PICK:
                if task_id == 0:
                    self._publish_tts('기어 번호를 말해주세요. 예: 기어 1번 픽')
                    ok = False
                    silent_fail = True
                elif self._holding:
                    self._publish_tts(f'이미 기어 {self._held_gear}번을 파지 중입니다.')
                    ok = False
                    silent_fail = True
                else:
                    task = GEAR_TASKS[task_id - 1]
                    self._publish_tts(f'기어 {task_id}번 픽을 시작합니다.')
                    self._set_state(f'PICKING({task_id})')
                    ok = self._run_pick(logger, task)
                    if ok:
                        self._held_gear = task_id
                        self._set_state(f'HOLDING({task_id})')
                    else:
                        self._set_state('ERROR')

            elif cmd == VoiceCommand.CMD_PLACE:
                if not self._holding:
                    self._publish_tts('먼저 픽을 해주세요.')
                    ok = False
                    silent_fail = True
                elif task_id == 0:
                    self._publish_tts('기어 번호를 말해주세요. 예: 기어 1번 플레이스')
                    ok = False
                    silent_fail = True
                else:
                    task = GEAR_TASKS[task_id - 1]
                    self._publish_tts(f'기어 {task_id}번 플레이스를 시작합니다.')
                    self._set_state(f'PLACING({task_id})')
                    ok = self._run_place(logger, task)
                    if ok:
                        self._held_gear = 0
                        self._set_state('IDLE')
                    else:
                        self._set_state('ERROR')

            elif cmd == VoiceCommand.CMD_PICKPLACE:
                tasks = self._resolve_tasks(task_id)
                total = len(tasks)
                task_str = f"기어 {task_id}번" if task_id > 0 else f"전체 {total}개"
                self._publish_tts(f'{task_str} 픽 앤 플레이스를 시작합니다.')
                ok = self._run_pickplace_sequence(logger, tasks, use_wiggle)
                self._set_state('IDLE')

            elif cmd == VoiceCommand.CMD_GRIPPER_OPEN:
                self._publish_tts('그리퍼를 엽니다.')
                ok = self._set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N)

            elif cmd == VoiceCommand.CMD_GRIPPER_CLOSE:
                self._publish_tts('그리퍼를 닫습니다.')
                ok = self._set_gripper(GRIPPER_CLOSE_WIDTH_MM, GRIPPER_FORCE_N)

            elif cmd == VoiceCommand.CMD_JOG:
                prev_state = self._robot_state
                self._set_state('JOGGING')
                ok = self._jog(msg.direction)
                self._set_state(prev_state)

            if self._is_cancelled():
                result_text = f'{msg.message} 취소됨'
            else:
                result_text = f'{msg.message} 완료' if ok else f'{msg.message} 실패'

            logger.info(f'===== {result_text} =====')
            if not silent_fail:
                self._publish_tts(result_text + '.')
            self._publish_status(result_text)

    # ────────────────────────────────────────────
    #   작업 목록 결정 (task_id=0: 전체, 1~4: 특정)
    # ────────────────────────────────────────────
    def _resolve_tasks(self, task_id: int) -> list:
        if task_id == 0:
            return list(GEAR_TASKS)
        idx = task_id - 1  # 0-based
        if 0 <= idx < len(GEAR_TASKS):
            return [GEAR_TASKS[idx]]
        self.get_logger().error(f"task_id={task_id} 범위 초과 (1~{len(GEAR_TASKS)})")
        return [GEAR_TASKS[-1]]  # fallback: 마지막 기어

    # ────────────────────────────────────────────
    #   Pick & Place 전체 시퀀스
    # ────────────────────────────────────────────
    def _run_pickplace_sequence(self, logger, tasks: list, use_wiggle: bool) -> bool:
        total = len(tasks)
        success = True

        for i, task in enumerate(tasks):
            gear_num = i + 1
            is_last  = (i == total - 1)
            logger.info(f"── 기어 {gear_num}/{total} 시작 ──")

            if self._is_cancelled():
                break

            # 1. HOME
            if not self._move_home():
                logger.error(f"기어{gear_num}: HOME 이동 실패")
                success = False
                break

            if self._is_cancelled():
                break

            # 2. APPROACH (pick 위)
            if not self._move_pose_raw(task["pick"], z_offset=self._approach):
                logger.error(f"기어{gear_num}: APPROACH 실패")
                success = False
                break

            if self._is_cancelled():
                break

            # 3. PICK
            if not self._move_pose_raw(task["pick"]):
                logger.error(f"기어{gear_num}: PICK 이동 실패")
                success = False
                break
            self._set_gripper(GRIPPER_CLOSE_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
            self._holding = True
            logger.info(f"기어{gear_num}: 그리퍼 CLOSE")

            if self._is_cancelled():
                break

            # 4. RETREAT (pick 위로)
            if not self._move_pose_raw(task["pick"], z_offset=self._approach):
                logger.error(f"기어{gear_num}: RETREAT 실패")
                success = False
                break

            if self._is_cancelled():
                break

            # 5. TRANSFER (place 위)
            if not self._move_pose_raw(task["place"], z_offset=self._approach):
                logger.error(f"기어{gear_num}: TRANSFER 실패")
                success = False
                break

            if self._is_cancelled():
                break

            # 6. WIGGLE (마지막 기어 + use_wiggle)
            if is_last and use_wiggle:
                logger.info(f"기어{gear_num}: 위글 시작")
                self._do_wiggle(task["place"])

                if self._is_cancelled():
                    break

            # 7. PLACE
            if not self._move_pose_raw(task["place"]):
                logger.error(f"기어{gear_num}: PLACE 이동 실패")
                success = False
                break
            self._set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
            self._holding = False
            logger.info(f"기어{gear_num}: 그리퍼 OPEN")

            # 8. PLACE RETREAT
            if not self._move_pose_raw(task["place"], z_offset=self._approach):
                logger.error(f"기어{gear_num}: PLACE RETREAT 실패")
                success = False
                break

            logger.info(f"기어 {gear_num}/{total} 완료")

        # 취소 또는 완료 후 홈 복귀
        self._move_home()
        return success and not self._is_cancelled()

    # ────────────────────────────────────────────
    #   단독 픽 시퀀스 (CMD_PICK)
    #   성공 시 _holding=True는 워커에서 세팅
    # ────────────────────────────────────────────
    def _run_pick(self, logger, task: dict) -> bool:
        if not self._move_pose_raw(task["pick"], z_offset=self._approach):
            return False
        if not self._move_pose_raw(task["pick"]):
            return False
        self._set_gripper(GRIPPER_CLOSE_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
        self._holding = True
        logger.info("그리퍼 CLOSE")
        return self._move_pose_raw(task["pick"], z_offset=self._approach)

    # ────────────────────────────────────────────
    #   단독 플레이스 시퀀스 (CMD_PLACE)
    #   성공 시 _holding=False는 워커에서 세팅
    # ────────────────────────────────────────────
    def _run_place(self, logger, task: dict) -> bool:
        if not self._move_pose_raw(task["place"], z_offset=self._approach):
            return False
        if not self._move_pose_raw(task["place"]):
            return False
        self._set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
        self._holding = False
        logger.info("그리퍼 OPEN")
        return self._move_pose_raw(task["place"], z_offset=self._approach)

    # ────────────────────────────────────────────
    #   위글 동작
    # ────────────────────────────────────────────
    def _do_wiggle(self, task_side: dict):
        pos = task_side["pos"]
        ori = task_side["ori"]

        wiggle_ps = PoseStamped()
        wiggle_ps.header.frame_id = BASE_FRAME
        wiggle_ps.pose.position.x = pos[0]
        wiggle_ps.pose.position.y = pos[1]
        wiggle_ps.pose.position.z = WIGGLE_Z

        base_q = (ori[3], ori[0], ori[1], ori[2])  # (w, x, y, z)
        yaw_rad = math.radians(WIGGLE_YAW_DEG)

        for _ in range(WIGGLE_COUNT):
            for sign in (+1, -1):
                if self._is_cancelled():
                    return
                rot = _yaw_quat(sign * yaw_rad)
                w, x, y, z = _quat_mul(base_q, rot)
                wiggle_ps.pose.orientation.w = w
                wiggle_ps.pose.orientation.x = x
                wiggle_ps.pose.orientation.y = y
                wiggle_ps.pose.orientation.z = z
                self._move_pose_stamped(wiggle_ps)

        # 원래 orientation 복귀
        wiggle_ps.pose.orientation.x = ori[0]
        wiggle_ps.pose.orientation.y = ori[1]
        wiggle_ps.pose.orientation.z = ori[2]
        wiggle_ps.pose.orientation.w = ori[3]
        self._move_pose_stamped(wiggle_ps)

    # ────────────────────────────────────────────
    #   방향 이동 (Jog)
    # ────────────────────────────────────────────
    def _jog(self, direction: int) -> bool:
        """현재 EE 위치에서 direction 방향으로 JOG_OFFSET만큼 이동."""
        return self._motion_pool.submit(self._jog_impl, direction).result()

    def _jog_impl(self, direction: int) -> bool:
        if self._is_cancelled():
            return False
        log = get_logger('stt_pick_and_place.motion')

        # ── 현재 EE 포즈 획득 ────────────────────────────
        try:
            with self._robot.get_planning_scene_monitor().read_only() as scene:
                state = scene.current_state
                state.update()
                tf = state.get_frame_transform(EE_LINK)  # 4×4 numpy array
        except Exception as e:
            log.error(f"현재 EE 상태 조회 실패: {e}")
            return False

        cx = float(tf[0, 3])
        cy = float(tf[1, 3])
        cz = float(tf[2, 3])
        qx, qy, qz, qw = _rot_matrix_to_quat(tf[:3, :3])

        # ── 방향 → 오프셋 ─────────────────────────────────
        delta = {
            VoiceCommand.DIR_FORWARD:  ( JOG_OFFSET,  0.0,         0.0),
            VoiceCommand.DIR_BACKWARD: (-JOG_OFFSET,  0.0,         0.0),
            VoiceCommand.DIR_LEFT:     ( 0.0,          JOG_OFFSET,  0.0),
            VoiceCommand.DIR_RIGHT:    ( 0.0,         -JOG_OFFSET,  0.0),
            VoiceCommand.DIR_UP:       ( 0.0,          0.0,         JOG_OFFSET),
            VoiceCommand.DIR_DOWN:     ( 0.0,          0.0,        -JOG_OFFSET),
        }
        dx, dy, dz = delta.get(direction, (0.0, 0.0, 0.0))

        tx, ty, tz = _clamp(cx + dx, cy + dy, cz + dz, log)
        log.info(
            f"JOG dir={direction}  offset={JOG_OFFSET*100:.1f}cm  "
            f"({cx:.3f},{cy:.3f},{cz:.3f}) → ({tx:.3f},{ty:.3f},{tz:.3f})"
        )

        ps = PoseStamped()
        ps.header.frame_id    = BASE_FRAME
        ps.pose.position.x    = tx
        ps.pose.position.y    = ty
        ps.pose.position.z    = tz
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE_LINK)
        return self._plan_and_execute(self._pilz_params)

    # ────────────────────────────────────────────
    #   모션 실행 (motion_pool 직렬화)
    # ────────────────────────────────────────────
    def _move_home(self) -> bool:
        return self._motion_pool.submit(self._move_home_impl).result()

    def _move_home_impl(self) -> bool:
        if self._is_cancelled():
            return False
        home_state = RobotState(self._robot_model)
        home_state.joint_positions = HOME_JOINTS_RAD
        home_state.update()
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=home_state)
        return self._plan_and_execute(self._home_params)

    def _move_pose_raw(self, task_side: dict, z_offset: float = 0.0) -> bool:
        pos = task_side["pos"]
        ori = task_side["ori"]
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x = pos[0]
        ps.pose.position.y = pos[1]
        ps.pose.position.z = pos[2] + z_offset
        ps.pose.orientation.x = ori[0]
        ps.pose.orientation.y = ori[1]
        ps.pose.orientation.z = ori[2]
        ps.pose.orientation.w = ori[3]
        return self._move_pose_stamped(ps)

    def _move_pose_stamped(self, ps: PoseStamped) -> bool:
        return self._motion_pool.submit(self._move_pose_impl, ps).result()

    def _move_pose_impl(self, ps: PoseStamped) -> bool:
        if self._is_cancelled():
            return False
        log = get_logger('stt_pick_and_place.motion')
        sx, sy, sz = _clamp(
            ps.pose.position.x,
            ps.pose.position.y,
            ps.pose.position.z,
            log,
        )
        ps.pose.position.x = sx
        ps.pose.position.y = sy
        ps.pose.position.z = sz
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE_LINK)
        return self._plan_and_execute(self._pilz_params)

    def _plan_and_execute(self, params) -> bool:
        log = get_logger('stt_pick_and_place.motion')
        if self._is_cancelled():
            log.info("취소 플래그 감지 — 플래닝 건너뜀")
            return False
        result = self._arm.plan(parameters=params)
        if not result:
            log.error("Planning failed")
            return False
        if self._is_cancelled():
            log.info("취소 플래그 감지 — 실행 건너뜀")
            return False
        self._robot.execute(
            group_name=GROUP_NAME,
            robot_trajectory=result.trajectory,
            blocking=True,
        )
        return True

    # ────────────────────────────────────────────
    #   그리퍼 제어
    # ────────────────────────────────────────────
    def _set_gripper(self, width_mm: float, force_n: float, blocking: bool = False) -> bool:
        if self._gripper is None:
            self.get_logger().info(
                f"[시뮬] 그리퍼: width={width_mm}mm force={force_n}N"
            )
            if blocking:
                time.sleep(1.0)
            return True
        try:
            self._gripper.move_gripper(
                width_val=int(width_mm * 10),
                force_val=int(force_n  * 10),
            )
            if blocking:
                time.sleep(1.0)
            return True
        except Exception as e:
            self.get_logger().error(f"그리퍼 명령 실패: {e}")
            return False

    # ────────────────────────────────────────────
    #   취소 플래그
    # ────────────────────────────────────────────
    def _is_cancelled(self) -> bool:
        return self._cancel_event.is_set()

    # ────────────────────────────────────────────
    #   로봇 상태 관리
    # ────────────────────────────────────────────
    def _set_state(self, state: str):
        self._robot_state = state
        self.get_logger().info(f'[STATE] {state}')

    def _publish_state(self):
        """2Hz 타이머 콜백 — /robot_state 퍼블리시."""
        msg = String()
        held = f'  held_gear={self._held_gear}' if self._holding else ''
        msg.data = f'{self._robot_state}{held}'
        self._state_pub.publish(msg)

    # ────────────────────────────────────────────
    #   퍼블리시 헬퍼
    # ────────────────────────────────────────────
    def _publish_tts(self, text: str):
        msg = String()
        msg.data = text
        self._tts_pub.publish(msg)

    def _publish_status(self, text: str):
        msg = String()
        msg.data = text
        self._status_pub.publish(msg)


# ════════════════════════════════════════════════════
#   엔트리포인트
# ════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    node = SttPickAndPlaceNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    node.get_logger().info('음성 명령 대기 중 ... (Ctrl+C 종료)')
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node._robot.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
