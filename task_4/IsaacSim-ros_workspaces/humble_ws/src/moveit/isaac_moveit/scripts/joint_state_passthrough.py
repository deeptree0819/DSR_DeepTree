#!/usr/bin/env python3
"""
joint_state_passthrough.py — Isaac Sim Physical Inspector ↔ MoveIt2 동기화

/manual_mode (std_msgs/Bool) 토픽으로 모드 전환:
  False (기본) : MoveIt2 제어 모드
  True         : 수동 모드
                  - /tmp/isaac_manual_mode 파일 생성
                    → Isaac Sim 이 OnImpulseEventCtrl 을 정지
                  - [sync_to_isaac=True] echo 먼저 시작 → JSB 비활성화 (순서 중요)
                    → /joint_states 가 끊기지 않으므로 수동 모드 중에도 플래닝 가능
                    → Physical Inspector 조작이 MoveIt2 에 즉시 반영

수동 모드 종료 (sync_to_isaac=True):
  - dsr_moveit_controller / rg2_gripper_controller 에 현재 Isaac Sim 위치 hold 궤적 전송
    → dsr_moveit_controller 의 hold point 가 B 로 갱신됨
  - 플래그 파일 제거 → OnImpulseEventCtrl 재개
    → TopicBasedSystem 이 B 를 /isaac_joint_commands 로 전송 → snap-back 없음
  - JSB 재활성화 확인 후 echo 중단 (역시 끊김 없음)

수동 모드 활성화:
  ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: true"

MoveIt2 모드 복귀:
  ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: false"
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from controller_manager_msgs.srv import SwitchController
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


ECHO_RATE_HZ     = 50.0
MANUAL_MODE_FLAG = "/tmp/isaac_manual_mode"

# MoveIt2가 추적하는 관절만 /joint_states 로 전달 (mimic joint 제외)
_TRACKED_JOINTS = frozenset([
    'joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6',
    'rg2_finger_joint',
])

# 컨트롤러별 담당 관절 (sync 용)
_ARM_JOINTS     = frozenset(['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6'])
_GRIPPER_JOINTS = frozenset(['rg2_finger_joint'])


class JointStatePassthrough(Node):

    def __init__(self):
        super().__init__('joint_state_passthrough')

        self._manual_mode  = False
        self._latest_js: JointState = None
        self._sync_pending = 0
        self._cb = ReentrantCallbackGroup()

        # sync_to_isaac=True : Isaac 모드 전용
        #   - 수동 모드에서 JSB 비활성화 + echo 로 /joint_states 직접 발행
        #   - 수동 모드 종료 시 컨트롤러 hold point 동기화 (snap-back 방지)
        # sync_to_isaac=False : real / virtual / mock 모드
        #   - JSB 항상 활성, DRHWInterface 가 실물 로봇 상태를 제공
        self._sync_to_isaac: bool = (
            self.declare_parameter('sync_to_isaac', False).value
        )

        # ── 구독 ─────────────────────────────────────────────────────
        self.create_subscription(
            JointState, '/isaac_joint_states',
            self._on_joint_state, 10,
            callback_group=self._cb,
        )
        self.create_subscription(
            Bool, '/manual_mode',
            self._on_manual_mode, 10,
            callback_group=self._cb,
        )

        # ── /joint_states 퍼블리셔 (Isaac 모드 수동 구간 전용) ────────
        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)

        # ── SwitchController (JSB 활성/비활성) ───────────────────────
        self._switch_ctrl = self.create_client(
            SwitchController, '/controller_manager/switch_controller',
        )

        # ── FollowJointTrajectory 액션 클라이언트 (sync_to_isaac 전용) ─
        if self._sync_to_isaac:
            self._arm_ac = ActionClient(
                self, FollowJointTrajectory,
                '/dsr_moveit_controller/follow_joint_trajectory',
                callback_group=self._cb,
            )
            self._gripper_ac = ActionClient(
                self, FollowJointTrajectory,
                '/rg2_gripper_controller/follow_joint_trajectory',
                callback_group=self._cb,
            )

        # ── Echo 타이머 (기본 정지) ──────────────────────────────────
        self._echo_timer = self.create_timer(
            1.0 / ECHO_RATE_HZ,
            self._echo_tick,
            callback_group=self._cb,
        )
        self._echo_timer.cancel()

        # 시작 시 플래그 파일 초기화
        if os.path.exists(MANUAL_MODE_FLAG):
            os.remove(MANUAL_MODE_FLAG)

        self.get_logger().info(
            'JointStatePassthrough 준비 완료 (sync_to_isaac=%s)\n'
            '  수동 모드: ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: true"\n'
            '  MoveIt 모드: ros2 topic pub --once /manual_mode std_msgs/msg/Bool "data: false"'
            % self._sync_to_isaac
        )

    # ──────────────────────────────────────────────────────────────
    # 구독 콜백
    # ──────────────────────────────────────────────────────────────

    def _on_joint_state(self, msg: JointState):
        self._latest_js = msg

    def _on_manual_mode(self, msg: Bool):
        if msg.data == self._manual_mode:
            return
        self._manual_mode = msg.data
        if self._manual_mode:
            self._enter_manual_mode()
        else:
            self._enter_moveit_mode()

    # ──────────────────────────────────────────────────────────────
    # 수동 모드 진입
    # ──────────────────────────────────────────────────────────────

    def _enter_manual_mode(self):
        # 1. 파일 플래그 생성 → Isaac Sim OnImpulseEventCtrl 정지
        with open(MANUAL_MODE_FLAG, 'w') as f:
            f.write("1")

        if self._sync_to_isaac:
            # 2. ★ echo 먼저 시작 ★
            #    JSB 비활성화보다 앞서 실행 → /joint_states 가 끊기지 않음
            #    → 수동 모드 중에도 MoveIt2 플래닝 정상 작동
            self._echo_timer.reset()

            # 3. JSB 비활성화 (비동기 — echo 가 이미 /joint_states 를 커버)
            #    JSB 가 명령 위치(원위치)를 퍼블리시하던 것을 제거
            #    → echo 만 남으므로 "원위치 ↔ Isaac 위치" 진동 없음
            self._switch_jsb(deactivate=True, done_cb=lambda ok:
                self.get_logger().warn('JSB 비활성화 실패 — 진동이 남을 수 있습니다.')
                if not ok else None
            )

        self.get_logger().info(
            '수동 모드 활성 — Isaac Sim Physical Inspector로 joint를 조작하세요.'
        )

    # ──────────────────────────────────────────────────────────────
    # MoveIt2 모드 복귀
    # ──────────────────────────────────────────────────────────────

    def _enter_moveit_mode(self):
        if self._sync_to_isaac and self._latest_js is not None:
            # 플래그 파일 제거 전에 컨트롤러 hold point 를 현재 Isaac Sim 위치로 동기화
            self._sync_controllers_then_exit()
        else:
            self._do_exit_manual()

    def _sync_controllers_then_exit(self):
        js = self._latest_js

        def _make_goal(joint_set):
            names = [n for n in js.name if n in joint_set]
            pos   = [p for n, p in zip(js.name, js.position) if n in joint_set]
            if not names:
                return None
            pt = JointTrajectoryPoint()
            pt.positions       = pos
            pt.time_from_start = Duration(sec=0, nanosec=500_000_000)  # 0.5 s

            traj = JointTrajectory()
            traj.joint_names = names
            traj.points      = [pt]

            goal = FollowJointTrajectory.Goal()
            goal.trajectory = traj
            return goal

        arm_goal     = _make_goal(_ARM_JOINTS)
        gripper_goal = _make_goal(_GRIPPER_JOINTS)

        self._sync_pending = 0
        pairs = [(self._arm_ac, arm_goal), (self._gripper_ac, gripper_goal)]

        for ac, goal in pairs:
            if goal is None:
                continue
            if not ac.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('액션 서버 없음 — 해당 컨트롤러 동기화 건너뜀')
                continue
            self._sync_pending += 1
            future = ac.send_goal_async(goal)
            future.add_done_callback(self._on_sync_goal_sent)

        if self._sync_pending == 0:
            self.get_logger().warn('동기화할 컨트롤러 없음 — 바로 모드 종료')
            self._do_exit_manual()

    def _on_sync_goal_sent(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('hold 궤적 거부됨')
            self._sync_pending -= 1
            if self._sync_pending <= 0:
                self._do_exit_manual()
            return
        handle.get_result_async().add_done_callback(self._on_sync_done)

    def _on_sync_done(self, _future):
        self._sync_pending -= 1
        if self._sync_pending <= 0:
            self._do_exit_manual()

    def _do_exit_manual(self):
        # 플래그 파일 제거 → Isaac Sim OnImpulseEventCtrl 재개
        # 이 시점에서 dsr_moveit_controller 는 B 를 hold 중
        # → TopicBasedSystem 이 B 를 /isaac_joint_commands 로 전송 → snap-back 없음
        if os.path.exists(MANUAL_MODE_FLAG):
            os.remove(MANUAL_MODE_FLAG)

        if self._sync_to_isaac:
            # JSB 재활성화 후 echo 중단
            # → JSB 가 확실히 살아난 뒤에 echo 를 끄므로 /joint_states 끊김 없음
            self._switch_jsb(deactivate=False, done_cb=self._on_jsb_reactivated)
        else:
            self.get_logger().info('MoveIt2 모드 복귀 — Plan & Execute를 사용하세요.')

    def _on_jsb_reactivated(self, ok: bool):
        # JSB 가 /joint_states 를 다시 발행하기 시작한 뒤 echo 중단
        self._echo_timer.cancel()
        if ok:
            self.get_logger().info('MoveIt2 모드 복귀 — Plan & Execute를 사용하세요.')
        else:
            self.get_logger().warn(
                'JSB 재활성화 실패 — /joint_states 가 발행되지 않을 수 있습니다.'
            )

    # ──────────────────────────────────────────────────────────────
    # Echo 타이머 — /joint_states 직접 발행 (Isaac 수동 모드 전용)
    # ──────────────────────────────────────────────────────────────

    def _echo_tick(self):
        if self._latest_js is None:
            return
        now   = self.get_clock().now().to_msg()
        pairs = [
            (n, p)
            for n, p in zip(self._latest_js.name, self._latest_js.position)
            if n in _TRACKED_JOINTS
        ]
        if not pairs:
            return
        js = JointState()
        js.header.stamp = now
        js.name, js.position = zip(*pairs)
        js.name     = list(js.name)
        js.position = list(js.position)
        self._js_pub.publish(js)

    # ──────────────────────────────────────────────────────────────
    # SwitchController 헬퍼
    # ──────────────────────────────────────────────────────────────

    def _switch_jsb(self, deactivate: bool, done_cb):
        if not self._switch_ctrl.wait_for_service(timeout_sec=3.0):
            self.get_logger().warn('switch_controller 서비스 없음 — JSB 전환 건너뜀')
            done_cb(False)
            return

        req = SwitchController.Request()
        if deactivate:
            req.deactivate_controllers = ['joint_state_broadcaster']
        else:
            req.activate_controllers = ['joint_state_broadcaster']
        req.strictness = SwitchController.Request.BEST_EFFORT

        future = self._switch_ctrl.call_async(req)
        future.add_done_callback(
            lambda f: done_cb((not f.cancelled()) and f.result().ok)
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointStatePassthrough()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if os.path.exists(MANUAL_MODE_FLAG):
            os.remove(MANUAL_MODE_FLAG)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
