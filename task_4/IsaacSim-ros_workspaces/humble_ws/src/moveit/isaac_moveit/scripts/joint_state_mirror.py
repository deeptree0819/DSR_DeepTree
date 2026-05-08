#!/usr/bin/env python3
"""
joint_state_mirror.py — 실물 로봇 joint 상태를 Isaac Sim으로 미러링

/joint_states  →  /isaac_joint_commands

mode:=real 또는 mode:=virtual 로 MoveIt2를 실행하면서
Isaac Sim도 함께 실행할 때 사용합니다.

MoveIt2가 실물 로봇을 제어하면 Isaac Sim 시뮬레이션도 동일하게 따라 움직입니다.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class JointStateMirror(Node):

    def __init__(self):
        super().__init__('joint_state_mirror')

        self._pub = self.create_publisher(JointState, '/isaac_joint_commands', 10)

        self.create_subscription(
            JointState, '/joint_states',
            self._on_joint_states, 10,
        )

        self.get_logger().info(
            'JointStateMirror 시작: /joint_states → /isaac_joint_commands\n'
            '  Isaac Sim이 실물 로봇의 움직임을 따라갑니다.'
        )

    def _on_joint_states(self, msg: JointState):
        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name     = msg.name
        cmd.position = list(msg.position)
        self._pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = JointStateMirror()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
