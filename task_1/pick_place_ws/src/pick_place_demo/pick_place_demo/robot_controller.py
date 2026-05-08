#!/usr/bin/env python3
"""
robot_controller.py — 기본 제어 노드 (과업 a-1)

MoveItPy + OnRobot 그리퍼를 소유하는 기반 노드.
 - 토픽: /robot_status, /gripper_state  (주기 발행)
 - 서비스: /set_gripper, /get_task_list
 - 공개 메서드: move_to_pose, move_to_home, set_gripper, update_task_state
   └ 같은 프로세스의 PickPlaceServer가 직접 호출한다.

원본: dsr_practice/gear_assembly.py 의 로직을 Node 클래스로 재구성.
"""

import concurrent.futures
import math
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseStamped

from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy, PlanRequestParameters

from pick_place_interfaces.msg import TaskStatus, GripperState
from pick_place_interfaces.srv import SetGripper, GetTaskList

from .onrobot import RG


GROUP_NAME = "manipulator"
BASE_FRAME = "base_link"
EE_LINK = "link_6"

HOME_JOINTS_RAD = {
    "joint_1": math.radians(0.0),
    "joint_2": math.radians(0.0),
    "joint_3": math.radians(90.0),
    "joint_4": math.radians(0.0),
    "joint_5": math.radians(90.0),
    "joint_6": math.radians(0.0),
}
# 그리퍼 (raw 단위: 1/10 mm, 1/10 N)
GRIPPER_NAME = "rg2"
TOOLCHARGER_IP = "192.168.1.1"
TOOLCHARGER_PORT = 502
GRIPPER_OPEN_WIDTH_MM = 50.0
GRIPPER_CLOSE_WIDTH_MM = 20.0
GRIPPER_FORCE_N = 20.0

# ====== 안전 작업 영역 정의 (base_link 기준) ======
SAFE_X_MIN = 0.0      # x는 0 이상
SAFE_Y_MIN = -0.3     # y 하한
SAFE_Y_MAX = 0.3      # y 상한
SAFE_Z_MIN = 0.27     # z는 이 값보다 낮아지면 안 됨
# ==================================================


def clamp_to_safe_workspace(x: float, y: float, z: float, logger):
    """안전 작업 영역으로 (x, y, z) 클램핑"""
    safe_x = x
    safe_y = y
    safe_z = z

    if safe_x < SAFE_X_MIN:
        logger.warning(
            f"Requested x ({safe_x:.3f} m) is below safety limit "
            f"({SAFE_X_MIN:.3f} m). Clamping to SAFE_X_MIN."
        )
        safe_x = SAFE_X_MIN

    if safe_y < SAFE_Y_MIN:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is below safety limit "
            f"({SAFE_Y_MIN:.3f} m). Clamping to SAFE_Y_MIN."
        )
        safe_y = SAFE_Y_MIN
    elif safe_y > SAFE_Y_MAX:
        logger.warning(
            f"Requested y ({safe_y:.3f} m) is above safety limit "
            f"({SAFE_Y_MAX:.3f} m). Clamping to SAFE_Y_MAX."
        )
        safe_y = SAFE_Y_MAX

    if safe_z < SAFE_Z_MIN:
        logger.warning(
            f"Requested z ({safe_z:.3f} m) is below safety limit "
            f"({SAFE_Z_MIN:.3f} m). Clamping to SAFE_Z_MIN."
        )
        safe_z = SAFE_Z_MIN

    return safe_x, safe_y, safe_z

# 기어 작업 목록
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

APPROACH_OFFSET = 0.05
WIGGLE_Z = 0.295
WIGGLE_YAW_DEG = 5.0
WIGGLE_COUNT = 3

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller")

        self.get_logger().info("=== RobotController 시작 ===")

        # ---- 하드웨어 / MoveIt ----
        self._gripper = RG(GRIPPER_NAME, TOOLCHARGER_IP, TOOLCHARGER_PORT)
        time.sleep(0.5)

        self._robot = MoveItPy(node_name="moveit_py")
        self._arm = self._robot.get_planning_component(GROUP_NAME)
        self._robot_model = self._robot.get_robot_model()

        self._home_params = self._make_plan_params("ompl", "RRTConnect", 0.2, 0.1)
        self._pilz_params = self._make_plan_params(
            "pilz_industrial_motion_planner", "PTP", 0.15, 0.1
        )

        # MoveItPy는 동일 스레드에서만 안전: 전용 단일 스레드로 모든 모션 직렬화
        self._motion_pool = concurrent.futures.ThreadPoolExecutor(
            max_workers=1, thread_name_prefix="motion"
        )

        # 취소 이벤트: 플래닝 완료 후 실행 직전에 체크
        self._cancel_event = threading.Event()

        # ---- 작업 상태 (PickPlaceServer 가 update_task_state 로 갱신) ----
        self._task_state = TaskStatus.STATE_IDLE
        self._task_step = TaskStatus.STEP_IDLE
        self._task_gear_index = 0
        self._task_total = len(GEAR_TASKS)
        self._task_progress = 0.0
        self._task_message = ""

        # ---- 목표 그리퍼 값 (토픽 퍼블리시용) ----
        self._target_width_mm = GRIPPER_OPEN_WIDTH_MM
        self._target_force_n = GRIPPER_FORCE_N

        # ---- 퍼블리셔 ----
        self._status_pub = self.create_publisher(TaskStatus, "/robot_status", 10)
        self._gripper_pub = self.create_publisher(GripperState, "/gripper_state", 10)

        # ---- 서비스 ----
        self.create_service(SetGripper, "/set_gripper", self._srv_set_gripper)
        self.create_service(GetTaskList, "/get_task_list", self._srv_get_task_list)

        # ---- 타이머: 10 Hz 상태 브로드캐스트 ----
        self.create_timer(0.1, self._publish_status)

        # 초기 그리퍼 열기
        self.set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N)

        self.get_logger().info("RobotController 초기화 완료")

    # ────────────────────────────────────────────
    #   내부 헬퍼
    # ────────────────────────────────────────────
    def _make_plan_params(self, pipeline, planner_id, vel, acc):
        p = PlanRequestParameters(self._robot)
        p.planning_pipeline = pipeline
        p.planner_id = planner_id
        p.max_velocity_scaling_factor = vel
        p.max_acceleration_scaling_factor = acc
        p.planning_time = 2.0
        return p

    def request_cancel(self):
        """외부에서 취소 요청 — 다음 플래닝 실행 직전에 중단."""
        self._cancel_event.set()

    def clear_cancel(self):
        """새 goal 시작 전 취소 플래그 초기화."""
        self._cancel_event.clear()

    def _plan_and_execute(self, plan_params):
        log = self.get_logger()
        if self._cancel_event.is_set():
            log.info("취소 플래그 감지 — 플래닝 건너뜀")
            return False
        log.info("Planning trajectory")
        plan_result = self._arm.plan(parameters=plan_params)
        if not plan_result:
            log.error("Planning failed")
            return False
        if self._cancel_event.is_set():
            log.info("취소 플래그 감지 — 실행 건너뜀")
            return False
        log.info("Executing plan")
        self._robot.execute(
            group_name=GROUP_NAME,
            robot_trajectory=plan_result.trajectory,
            blocking=True,
        )
        return True

    # ────────────────────────────────────────────
    #   공개 메서드 (PickPlaceServer 에서 호출)
    # ────────────────────────────────────────────
    def move_to_pose(self, pose: Pose, use_pilz: bool = True) -> bool:
        """Pose 목표로 이동. 안전영역 클램핑 포함."""
        return self._motion_pool.submit(self._move_to_pose_impl, pose, use_pilz).result()

    def _move_to_pose_impl(self, pose: Pose, use_pilz: bool) -> bool:
        sx, sy, sz = clamp_to_safe_workspace(
            pose.position.x, pose.position.y, pose.position.z, self.get_logger()
        )
        ps = PoseStamped()
        ps.header.frame_id = BASE_FRAME
        ps.pose.position.x = sx
        ps.pose.position.y = sy
        ps.pose.position.z = sz
        ps.pose.orientation = pose.orientation

        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(pose_stamped_msg=ps, pose_link=EE_LINK)
        params = self._pilz_params if use_pilz else self._home_params
        return self._plan_and_execute(params)

    def move_to_home(self) -> bool:
        """HOME 관절각으로 이동."""
        return self._motion_pool.submit(self._move_to_home_impl).result()

    def _move_to_home_impl(self) -> bool:
        home_state = RobotState(self._robot_model)
        home_state.joint_positions = HOME_JOINTS_RAD
        home_state.update()
        self._arm.set_start_state_to_current_state()
        self._arm.set_goal_state(robot_state=home_state)
        return self._plan_and_execute(self._home_params)

    def set_gripper(self, width_mm: float, force_n: float, blocking: bool = True) -> bool:
        """그리퍼 이동. width/force 는 mm / N 단위."""
        self._target_width_mm = width_mm
        self._target_force_n = force_n
        width_raw = int(width_mm * 10)
        force_raw = int(force_n * 10)
        try:
            self._gripper.move_gripper(width_val=width_raw, force_val=force_raw)
        except Exception as e:
            self.get_logger().error(f"그리퍼 명령 실패: {e}")
            return False
        if blocking:
            time.sleep(1.0)
        return True

    def update_task_state(self, *, state=None, step=None, gear_index=None,
                          progress=None, message=None):
        """PickPlaceServer 가 호출해 현재 작업 상태를 갱신."""
        if state is not None:
            self._task_state = state
        if step is not None:
            self._task_step = step
        if gear_index is not None:
            self._task_gear_index = gear_index
        if progress is not None:
            self._task_progress = progress
        if message is not None:
            self._task_message = message

    def get_task_count(self) -> int:
        return self._task_total

    def get_gear_task(self, idx: int):
        """1-based 인덱스로 GEAR_TASKS 조회."""
        return GEAR_TASKS[idx - 1]

    # ────────────────────────────────────────────
    #   타이머: 상태 브로드캐스트
    # ────────────────────────────────────────────
    def _publish_status(self):
        now = self.get_clock().now().to_msg()

        ts = TaskStatus()
        ts.header = Header(stamp=now, frame_id=BASE_FRAME)
        ts.state = self._task_state
        ts.total_gears = self._task_total
        ts.gear_index = self._task_gear_index
        ts.current_step = self._task_step
        ts.progress_pct = float(self._task_progress)
        ts.message = self._task_message
        self._status_pub.publish(ts)

        gs = GripperState()
        gs.header = Header(stamp=now, frame_id="gripper")
        try:
            gs.current_width_mm = float(self._gripper.get_width())
            sl = self._gripper.get_status()  # list[int] 길이 7
            gs.busy            = bool(sl[0])
            gs.grip_detected   = bool(sl[1])
            gs.safety1_pushed  = bool(sl[2])
            gs.safety1_trigged = bool(sl[3])
            gs.safety2_pushed  = bool(sl[4])
            gs.safety2_trigged = bool(sl[5])
            gs.safety_error    = bool(sl[6])
        except Exception as e:
            self.get_logger().warn(f"그리퍼 상태 읽기 실패: {e}", throttle_duration_sec=5)
        gs.target_width_mm = float(self._target_width_mm)
        gs.target_force_n = float(self._target_force_n)
        self._gripper_pub.publish(gs)

    # ────────────────────────────────────────────
    #   서비스 콜백
    # ────────────────────────────────────────────
    def _srv_set_gripper(self, request, response):
        self.get_logger().info(
            f"/set_gripper 요청: width={request.width_mm}mm force={request.force_n}N"
        )
        ok = self.set_gripper(request.width_mm, request.force_n, request.blocking)
        response.success = ok
        try:
            response.actual_width_mm = float(self._gripper.get_width())
        except Exception:
            response.actual_width_mm = 0.0
        response.message = "" if ok else "gripper command failed"
        return response

    def _srv_get_task_list(self, request, response):
        response.success = True
        response.total_count = len(GEAR_TASKS)

        for task in GEAR_TASKS:
            pick_pose = Pose()
            pick_pose.position.x, pick_pose.position.y, pick_pose.position.z = task["pick"]["pos"]
            (pick_pose.orientation.x, pick_pose.orientation.y,
             pick_pose.orientation.z, pick_pose.orientation.w) = task["pick"]["ori"]
            response.pick_poses.append(pick_pose)

            place_pose = Pose()
            place_pose.position.x, place_pose.position.y, place_pose.position.z = task["place"]["pos"]
            (place_pose.orientation.x, place_pose.orientation.y,
             place_pose.orientation.z, place_pose.orientation.w) = task["place"]["ori"]
            response.place_poses.append(place_pose)

        response.last_gear_wiggle = True
        response.wiggle_z = WIGGLE_Z
        response.wiggle_yaw_deg = WIGGLE_YAW_DEG
        response.wiggle_count = WIGGLE_COUNT
        response.message = f"{len(GEAR_TASKS)} tasks"
        return response


def main(args=None):
    """robot_controller 단독 실행용 엔트리포인트."""
    rclpy.init(args=args)
    node = RobotController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
