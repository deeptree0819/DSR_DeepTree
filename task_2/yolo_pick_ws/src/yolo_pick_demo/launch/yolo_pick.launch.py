"""
yolo_pick.launch.py
RealSense 드라이버 + yolo_pick_node 를 한 번에 실행.

사용법:
  # 기본 (yolo11n.pt)
  ros2 launch yolo_pick_demo yolo_pick.launch.py

  # 커스텀 가중치 지정
  ros2 launch yolo_pick_demo yolo_pick.launch.py model:=best.pt

  # 특정 클래스 ID만 픽 (0 = person, 39 = bottle ...)
  ros2 launch yolo_pick_demo yolo_pick.launch.py target_cls:=39

전제:
  - DSR 드라이버(dsr_bringup2)는 별도 터미널에서 미리 실행되어 있어야 함
  - RealSense rs_align_depth_launch.py 도 별도 또는 여기서 함께 실행

터미널 구성:
  [터미널 1] ros2 launch dsr_bringup2 dsr_bringup2_moveit.launch.py mode:=real model:=m0609 host:=192.168.1.1
  [터미널 2] ros2 launch realsense2_camera rs_align_depth_launch.py ...
  [터미널 3] ros2 launch yolo_pick_demo yolo_pick.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    args = [
        DeclareLaunchArgument(
            "model",
            default_value="yolo11n.pt",
            description="YOLO 가중치 파일 경로 (절대 경로 또는 ultralytics 기본 모델명)",
        ),
        DeclareLaunchArgument(
            "conf_thresh",
            default_value="0.5",
            description="YOLO 최소 신뢰도 (0.0 ~ 1.0)",
        ),
        DeclareLaunchArgument(
            "target_cls",
            default_value="-1",
            description="픽 대상 클래스 ID (-1 = 전체 클래스)",
        ),
        DeclareLaunchArgument(
            "auto_interval",
            default_value="3.0",
            description="자동 모드에서 픽 간격 [s]",
        ),
    ]

    yolo_pick_node = Node(
        package="yolo_pick_demo",
        executable="yolo_pick",
        name="yolo_pick_node",
        namespace="dsr01",
        output="screen",
        parameters=[{
            "yolo_model_path":     LaunchConfiguration("model"),
            "yolo_conf_thresh":    LaunchConfiguration("conf_thresh"),
            "yolo_target_cls":     LaunchConfiguration("target_cls"),
            "auto_pick_interval":  LaunchConfiguration("auto_interval"),
        }],
    )

    return LaunchDescription(args + [yolo_pick_node])
