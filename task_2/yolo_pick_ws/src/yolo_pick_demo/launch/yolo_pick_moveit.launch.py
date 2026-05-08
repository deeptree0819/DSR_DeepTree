"""
yolo_pick_moveit.launch.py
MoveIt 기반 YOLO Pick & Place 노드 실행.

MoveItPy 가 요구하는 robot_description / SRDF / kinematics / controllers 등
모든 MoveIt 설정을 함께 넘겨야 planning scene monitor 가 초기화된다.

전제:
  - DSR MoveIt 드라이버(dsr_bringup2_moveit)는 별도 터미널에서 미리 실행되어 있어야 함
  - RealSense rs_align_depth_launch.py 도 별도 터미널에서 실행

터미널 구성:
  [터미널 1] ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.1.1
  [터미널 2] ros2 launch realsense2_camera rs_align_depth_launch.py
  [터미널 3] ros2 launch yolo_pick_demo yolo_pick_moveit.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    # Doosan M0609 MoveIt 기본 설정 (URDF, SRDF, kinematics, controllers 등)
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic(file_path="config/dsr.srdf")
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    # MoveItPy 전용 파라미터 (planning pipelines, plan_request_params 등)
    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("yolo_pick_demo"), "config", "moveit_py.yaml"]
    )

    yolo_pick_moveit_node = Node(
        package="yolo_pick_demo",
        executable="yolo_pick_moveit",
        name="yolo_pick_moveit_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_py_params,
        ],
    )

    return LaunchDescription([yolo_pick_moveit_node])
