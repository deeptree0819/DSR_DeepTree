#!/usr/bin/env python3
"""
isaac_to_real_streamer.py — Isaac Sim 수동 조작 → 실물 로봇 streaming

수동 모드 진입 시 Isaac Sim 의 /isaac_joint_states 를 실물 로봇 컨트롤러의
joint_trajectory 토픽으로 짧은 궤적(lookahead 0.15s)으로 30 Hz 재발행.

이 노드는 Isaac Sim 외부의 시스템 Python (3.10) 에서 실행되어야 함.
Isaac Sim 번들 Python (3.11) 은 ROS2 Humble 의 rclpy C 확장과
ABI 가 맞지 않아 import 가 실패하기 때문.

실물 모드 (mode:=real) 에서만 의미가 있음. Isaac 단독 모드는
joint_state_passthrough.py (sync_to_isaac=True) 가 진입/종료 시 hold 궤적을
처리하므로 별도 streaming 이 필요하지 않음.
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


_ARM_JOINTS     = frozenset(['joint_1', 'joint_2', 'joint_3',
                              'joint_4', 'joint_5', 'joint_6'])
_GRIPPER_JOINTS = frozenset(['rg2_finger_joint'])
_STREAM_RATE_HZ = 30.0
_LOOKAHEAD_SEC  = 0.15  # 컨트롤러 보간 시간


class IsaacToRealStreamer(Node):

    def __init__(self):
        super().__init__('isaac_to_real_streamer')

        self._latest_js: JointState = None
        self._manual = False
        cb = ReentrantCallbackGroup()

        self.create_subscription(
            JointState, '/isaac_joint_states',
            self._on_js, 10, callback_group=cb,
        )
        self.create_subscription(
            Bool, '/manual_mode',
            self._on_manual, 10, callback_group=cb,
        )

        self._arm_pub = self.create_publisher(
            JointTrajectory, '/dsr_moveit_controller/joint_trajectory', 10,
        )
        self._gripper_pub = self.create_publisher(
            JointTrajectory, '/rg2_gripper_controller/joint_trajectory', 10,
        )

        self._timer = self.create_timer(
            1.0 / _STREAM_RATE_HZ,
            self._stream_tick,
            callback_group=cb,
        )
        self._timer.cancel()

        self.get_logger().info(
            'IsaacToRealStreamer 준비 완료 — /manual_mode true 시 실물 streaming 시작'
        )

    def _on_js(self, msg: JointState):
        self._latest_js = msg

    def _on_manual(self, msg: Bool):
        if msg.data == self._manual:
            return
        self._manual = msg.data
        if self._manual:
            self.get_logger().info('수동 모드 진입 → Isaac→실물 streaming 시작')
            self._timer.reset()
        else:
            self.get_logger().info(
                '수동 모드 종료 → streaming 중단 '
                '(passthrough 가 hold 동기화 후 MoveIt2 모드 복귀)'
            )
            self._timer.cancel()

    def _stream_tick(self):
        if self._latest_js is None or not self._manual:
            return

        sec  = int(_LOOKAHEAD_SEC)
        nsec = int((_LOOKAHEAD_SEC - sec) * 1e9)
        dur  = Duration(sec=sec, nanosec=nsec)
        stamp = self.get_clock().now().to_msg()

        def _make_traj(joint_set):
            names, pos = [], []
            for n, p in zip(self._latest_js.name, self._latest_js.position):
                if n in joint_set:
                    names.append(n)
                    pos.append(p)
            if not names:
                return None
            traj = JointTrajectory()
            traj.header.stamp = stamp
            traj.joint_names  = names
            pt = JointTrajectoryPoint()
            pt.positions       = pos
            pt.time_from_start = dur
            traj.points = [pt]
            return traj

        arm = _make_traj(_ARM_JOINTS)
        if arm is not None:
            self._arm_pub.publish(arm)

        gripper = _make_traj(_GRIPPER_JOINTS)
        if gripper is not None:
            self._gripper_pub.publish(gripper)


def main(args=None):
    rclpy.init(args=args)
    node = IsaacToRealStreamer()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
