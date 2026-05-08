#!/usr/bin/env python3
"""
pick_place_client.py — PickAndPlace 액션 클라이언트

사용법:
  # 전체 4개 기어 순서대로 실행 (기본)
  ros2 run pick_place_demo pick_place_client

  # 특정 기어 1개만 실행 (1~4)
  ros2 run pick_place_demo pick_place_client --id 2

  # 위글 비활성화
  ros2 run pick_place_demo pick_place_client --no-wiggle

  # 접근 오프셋 변경
  ros2 run pick_place_demo pick_place_client --approach 0.08
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from pick_place_interfaces.action import PickAndPlace
from pick_place_interfaces.srv import GetTaskList


class PickPlaceClient(Node):

    def __init__(self):
        super().__init__("pick_place_client")
        self._action_client = ActionClient(self, PickAndPlace, "/pick_and_place")
        self._task_cli = self.create_client(GetTaskList, "/get_task_list")

    # ────────────────────────────────────────────
    #   작업 목록 조회
    # ────────────────────────────────────────────
    def _fetch_task_list(self):
        """서버에서 등록된 기어 작업 목록을 가져온다."""
        if not self._task_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("/get_task_list 서비스를 찾을 수 없습니다")
            return None

        future = self._task_cli.call_async(GetTaskList.Request())
        rclpy.spin_until_future_complete(self, future)
        resp = future.result()

        if resp is None or not resp.success:
            self.get_logger().error("작업 목록 조회 실패")
            return None
        return resp

    # ────────────────────────────────────────────
    #   전체 기어 실행
    # ────────────────────────────────────────────
    def run_all(self, approach: float, use_wiggle: bool) -> bool:
        goal = PickAndPlace.Goal()
        goal.use_all_tasks = True
        goal.task_id = ""
        goal.approach_offset_m = approach
        goal.use_wiggle = use_wiggle
        goal.max_velocity_scale = 0.15
        return self._send_and_wait(goal)

    # ────────────────────────────────────────────
    #   단일 기어 실행
    # ────────────────────────────────────────────
    def run_single(self, gear_id: int, approach: float, use_wiggle: bool) -> bool:
        task_list = self._fetch_task_list()
        if task_list is None:
            return False

        total = task_list.total_count
        if not (1 <= gear_id <= total):
            self.get_logger().error(f"gear_id={gear_id} 범위 초과 (1~{total})")
            return False

        idx = gear_id - 1  # 0-based
        goal = PickAndPlace.Goal()
        goal.use_all_tasks = False
        goal.task_id = f"gear_{gear_id}"
        goal.pick_pose = task_list.pick_poses[idx]
        goal.place_pose = task_list.place_poses[idx]
        goal.approach_offset_m = approach
        goal.use_wiggle = use_wiggle
        goal.max_velocity_scale = 0.15
        return self._send_and_wait(goal)

    # ────────────────────────────────────────────
    #   공통 전송/대기 로직
    # ────────────────────────────────────────────
    def _send_and_wait(self, goal: PickAndPlace.Goal) -> bool:
        log = self.get_logger()
        log.info("액션 서버 대기 중...")
        self._action_client.wait_for_server()
        log.info(f"목표 전송: use_all={goal.use_all_tasks} task_id={goal.task_id!r}")

        send_future = self._action_client.send_goal_async(
            goal,
            feedback_callback=self._feedback_cb,
        )
        rclpy.spin_until_future_complete(self, send_future)

        goal_handle = send_future.result()
        if not goal_handle.accepted:
            log.error("목표가 거부되었습니다")
            return False

        log.info("목표 수락 — 실행 중...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result().result
        if result.success:
            log.info(
                f"[성공] {result.completed_gears}개 기어 완료 "
                f"({result.elapsed_sec:.1f}s) — {result.message}"
            )
        else:
            log.error(
                f"[실패] {result.completed_gears}개 기어 완료 후 중단 "
                f"— {result.message}"
            )
        return result.success

    # ────────────────────────────────────────────
    #   피드백 콜백
    # ────────────────────────────────────────────
    def _feedback_cb(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"  [기어 {fb.gear_index}/{fb.total_gears}] "
            f"{fb.current_step:<10s} "
            f"{fb.progress_pct:5.1f}%  {fb.message}"
        )


# ════════════════════════════════════════════════════
#   엔트리포인트
# ════════════════════════════════════════════════════
def main(args=None):
    parser = argparse.ArgumentParser(
        description="Pick & Place 액션 클라이언트",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "--id",
        type=int,
        default=0,
        metavar="N",
        help="실행할 기어 번호 (1~4). 생략 시 전체 4개 순서대로 실행",
    )
    parser.add_argument(
        "--no-wiggle",
        action="store_true",
        help="마지막 기어 위글 동작 비활성화",
    )
    parser.add_argument(
        "--approach",
        type=float,
        default=0.05,
        metavar="M",
        help="픽/플레이스 접근 z 오프셋 [m] (기본: 0.05)",
    )
    known, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args if ros_args else args)
    client = PickPlaceClient()

    use_wiggle = not known.no_wiggle

    try:
        if known.id == 0:
            success = client.run_all(approach=known.approach, use_wiggle=use_wiggle)
        else:
            success = client.run_single(
                gear_id=known.id, approach=known.approach, use_wiggle=use_wiggle
            )
    finally:
        client.destroy_node()
        rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
