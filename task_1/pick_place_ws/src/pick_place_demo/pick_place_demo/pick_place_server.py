#!/usr/bin/env python3
"""
pick_place_server.py — PickAndPlace 액션 서버

RobotController와 같은 프로세스에서 실행.
액션 goal을 받아 approach → pick → retreat → transfer → [wiggle] → place 시퀀스를 실행.
"""

import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Pose

from pick_place_interfaces.action import PickAndPlace
from pick_place_interfaces.msg import TaskStatus

from .robot_controller import (
    RobotController,
    GEAR_TASKS,
    APPROACH_OFFSET,
    WIGGLE_Z,
    WIGGLE_YAW_DEG,
    WIGGLE_COUNT,
    GRIPPER_OPEN_WIDTH_MM,
    GRIPPER_CLOSE_WIDTH_MM,
    GRIPPER_FORCE_N,
)


# ════════════════════════════════════════════════════
#   쿼터니언 유틸 (yaw 위글용)
# ════════════════════════════════════════════════════
def _yaw_quat(yaw_rad):
    """Z축 순수 yaw 회전 쿼터니언 (w, x, y, z)."""
    return (math.cos(yaw_rad / 2), 0.0, 0.0, math.sin(yaw_rad / 2))


def _quat_mul(q1, q2):
    """해밀턴 쿼터니언 곱 (w, x, y, z 순서)."""
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    return (
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
    )


def _make_pose(pos, ori) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = pos
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = ori
    return p


def _offset_z(pose: Pose, dz: float) -> Pose:
    """pose를 z축으로 dz만큼 오프셋한 새 Pose 반환."""
    p = Pose()
    p.position.x = pose.position.x
    p.position.y = pose.position.y
    p.position.z = pose.position.z + dz
    p.orientation = pose.orientation
    return p


# ════════════════════════════════════════════════════
#   PickPlaceServer Node
# ════════════════════════════════════════════════════
class PickPlaceServer(Node):

    def __init__(self, controller: RobotController):
        super().__init__("pick_place_server")
        self._ctrl = controller

        self._action_server = ActionServer(
            self,
            PickAndPlace,
            "/pick_and_place",
            execute_callback=self._execute_cb,
            goal_callback=self._goal_cb,
            cancel_callback=self._cancel_cb,
            callback_group=ReentrantCallbackGroup(),
        )
        self.get_logger().info("PickPlaceServer 준비 완료 (/pick_and_place)")

    # ────────────────────────────────────────────
    #   goal / cancel 콜백
    # ────────────────────────────────────────────
    def _goal_cb(self, goal_request):
        self.get_logger().info(
            f"목표 수신: task_id={goal_request.task_id!r} "
            f"use_all={goal_request.use_all_tasks}"
        )
        return GoalResponse.ACCEPT

    def _cancel_cb(self, goal_handle):
        self.get_logger().info("취소 요청 수신")
        self._ctrl.request_cancel()   # 플래닝/실행 직전 중단 전파
        return CancelResponse.ACCEPT

    # ────────────────────────────────────────────
    #   피드백 헬퍼
    # ────────────────────────────────────────────
    def _feedback(self, goal_handle, gear_index, total_gears, step, progress_pct, msg=""):
        fb = PickAndPlace.Feedback()
        fb.gear_index = gear_index
        fb.total_gears = total_gears
        fb.current_step = step
        fb.progress_pct = float(progress_pct)
        fb.gripper_busy = False
        fb.message = msg
        goal_handle.publish_feedback(fb)

        self._ctrl.update_task_state(
            step=step,
            gear_index=gear_index,
            progress=progress_pct,
            message=msg,
        )

    # ────────────────────────────────────────────
    #   액션 실행 콜백
    # ────────────────────────────────────────────
    def _execute_cb(self, goal_handle):
        log = self.get_logger()
        goal = goal_handle.request
        start_time = time.time()
        self._ctrl.clear_cancel()  # 이전 취소 플래그 초기화

        # 작업 목록 결정
        if goal.use_all_tasks:
            tasks = [
                {
                    "pick": _make_pose(t["pick"]["pos"], t["pick"]["ori"]),
                    "place": _make_pose(t["place"]["pos"], t["place"]["ori"]),
                }
                for t in GEAR_TASKS
            ]
        else:
            tasks = [{"pick": goal.pick_pose, "place": goal.place_pose}]

        total = len(tasks)
        approach = goal.approach_offset_m if goal.approach_offset_m > 0.0 else APPROACH_OFFSET

        # 진행률 계산: 기어당 스텝 = HOME + APPROACH + PICK + RETREAT + TRANSFER + PLACE = 6
        # 마지막 기어에 wiggle 활성화 시 +1
        steps_per_gear = 6
        total_steps = total * steps_per_gear + (1 if goal.use_wiggle else 0)
        done_steps = 0

        def pct():
            return done_steps / total_steps * 100.0

        self._ctrl.update_task_state(
            state=TaskStatus.STATE_RUNNING,
            step=TaskStatus.STEP_IDLE,
            gear_index=0,
            progress=0.0,
            message="작업 시작",
        )

        completed_gears = 0
        success = True

        for i, task in enumerate(tasks):
            gear_num = i + 1
            is_last = i == total - 1
            pick_pose: Pose = task["pick"]
            place_pose: Pose = task["place"]
            log.info(f"── 기어 {gear_num}/{total} 시작 ──")

            # 취소 체크 헬퍼 — goal_handle 플래그 또는 controller 이벤트 중 하나라도 세팅되면 취소
            def cancelled():
                if goal_handle.is_cancel_requested or self._ctrl._cancel_event.is_set():
                    log.info("취소 요청 감지")
                    return True
                return False

            # 1. HOME
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_HOME, pct(), f"기어{gear_num}: 홈 복귀")
            if not self._ctrl.move_to_home():
                log.error("HOME 이동 실패")
                success = False
                break
            done_steps += 1

            if cancelled(): break

            # 2. APPROACH — pick 위 approach 높이로 접근
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_APPROACH, pct(), f"기어{gear_num}: 픽 접근")
            if not self._ctrl.move_to_pose(_offset_z(pick_pose, approach)):
                log.error("APPROACH 이동 실패")
                success = False
                break
            done_steps += 1

            if cancelled(): break

            # 3. PICK — pick 위치로 내려가서 그리퍼 닫기
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_PICK, pct(), f"기어{gear_num}: 픽")
            if not self._ctrl.move_to_pose(pick_pose):
                log.error("PICK 이동 실패")
                success = False
                break
            self._ctrl.set_gripper(GRIPPER_CLOSE_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
            done_steps += 1

            if cancelled(): break

            # 4. RETREAT — pick 위로 상승
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_RETREAT, pct(), f"기어{gear_num}: 후퇴")
            if not self._ctrl.move_to_pose(_offset_z(pick_pose, approach)):
                log.error("RETREAT 이동 실패")
                success = False
                break
            done_steps += 1

            if cancelled(): break

            # 5. TRANSFER — place 위로 이동
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_TRANSFER, pct(), f"기어{gear_num}: 이송")
            if not self._ctrl.move_to_pose(_offset_z(place_pose, approach)):
                log.error("TRANSFER 이동 실패")
                success = False
                break
            done_steps += 1

            if cancelled(): break

            # 6. WIGGLE — 마지막 기어 + use_wiggle 활성 시
            if is_last and goal.use_wiggle:
                self._feedback(goal_handle, gear_num, total,
                               TaskStatus.STEP_WIGGLE, pct(), f"기어{gear_num}: 위글")
                self._do_wiggle(place_pose)
                done_steps += 1

                if cancelled(): break

            # 7. PLACE — place 위치로 내려가서 그리퍼 열기
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_PLACE, pct(), f"기어{gear_num}: 플레이스")
            if not self._ctrl.move_to_pose(place_pose):
                log.error("PLACE 이동 실패")
                success = False
                break
            self._ctrl.set_gripper(GRIPPER_OPEN_WIDTH_MM, GRIPPER_FORCE_N, blocking=True)
            done_steps += 1

            # 8. PLACE RETREAT — place 위로 상승 후 다음 기어로
            self._feedback(goal_handle, gear_num, total,
                           TaskStatus.STEP_RETREAT, pct(), f"기어{gear_num}: 플레이스 후퇴")
            if not self._ctrl.move_to_pose(_offset_z(place_pose, approach)):
                log.error("PLACE RETREAT 이동 실패")
                success = False
                break

            completed_gears += 1
            log.info(f"기어 {gear_num} 완료")

        # 취소 처리
        if goal_handle.is_cancel_requested or self._ctrl._cancel_event.is_set():
            self._ctrl.move_to_home()
            self._ctrl.update_task_state(
                state=TaskStatus.STATE_IDLE,
                step=TaskStatus.STEP_IDLE,
                message="취소됨",
            )
            result = PickAndPlace.Result()
            result.success = False
            result.completed_gears = completed_gears
            result.elapsed_sec = float(time.time() - start_time)
            result.message = "취소됨"
            goal_handle.canceled()
            return result

        # 종료 후 홈 복귀
        self._ctrl.move_to_home()

        elapsed = time.time() - start_time
        self._ctrl.update_task_state(
            state=TaskStatus.STATE_SUCCESS if success else TaskStatus.STATE_ERROR,
            step=TaskStatus.STEP_IDLE,
            gear_index=completed_gears,
            progress=100.0 if success else pct(),
            message="완료" if success else "오류 발생",
        )

        result = PickAndPlace.Result()
        result.success = success
        result.completed_gears = completed_gears
        result.elapsed_sec = float(elapsed)
        result.message = f"{completed_gears}/{total} 기어 완료 ({elapsed:.1f}s)"

        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result

    # ────────────────────────────────────────────
    #   위글 헬퍼
    # ────────────────────────────────────────────
    def _do_wiggle(self, base_place_pose: Pose):
        """place 위치에서 yaw ±WIGGLE_YAW_DEG 를 WIGGLE_COUNT 회 반복."""
        wiggle_pose = Pose()
        wiggle_pose.position.x = base_place_pose.position.x
        wiggle_pose.position.y = base_place_pose.position.y
        wiggle_pose.position.z = WIGGLE_Z

        base_q = (
            base_place_pose.orientation.w,
            base_place_pose.orientation.x,
            base_place_pose.orientation.y,
            base_place_pose.orientation.z,
        )
        yaw_rad = math.radians(WIGGLE_YAW_DEG)

        for _ in range(WIGGLE_COUNT):
            for sign in (+1, -1):
                rot = _yaw_quat(sign * yaw_rad)
                w, x, y, z = _quat_mul(base_q, rot)
                wiggle_pose.orientation.w = w
                wiggle_pose.orientation.x = x
                wiggle_pose.orientation.y = y
                wiggle_pose.orientation.z = z
                self._ctrl.move_to_pose(wiggle_pose)

        # 원래 orientation 복귀
        wiggle_pose.orientation = base_place_pose.orientation
        self._ctrl.move_to_pose(wiggle_pose)


# ════════════════════════════════════════════════════
#   엔트리포인트
# ════════════════════════════════════════════════════
def main(args=None):
    rclpy.init(args=args)

    controller = RobotController()
    server = PickPlaceServer(controller)

    executor = MultiThreadedExecutor()
    executor.add_node(controller)
    executor.add_node(server)

    try:
        executor.spin()
    finally:
        server.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
