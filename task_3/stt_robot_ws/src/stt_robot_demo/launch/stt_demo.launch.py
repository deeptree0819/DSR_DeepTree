"""
stt_demo.launch.py
STT → NLP → 로봇 실행 + TTS 피드백 4개 노드를 한 번에 실행.

실행:
  ros2 launch stt_robot_demo stt_demo.launch.py
  ros2 launch stt_robot_demo stt_demo.launch.py language:=en-US tts_lang:=en
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # ── MoveIt 설정 (Doosan M0609) ────────────────────────
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="m0609",
            package_name="dsr_moveit_config_m0609",
        )
        .robot_description()
        .robot_description_semantic()
        .robot_description_kinematics()
        .joint_limits()
        .trajectory_execution()
        .planning_scene_monitor()
        .sensors_3d()
        .to_moveit_configs()
    )

    moveit_py_params = PathJoinSubstitution(
        [FindPackageShare("stt_robot_demo"), "config", "moveit_py.yaml"]
    )

    keyword_map_path = PathJoinSubstitution(
        [FindPackageShare("stt_robot_demo"), "config", "keyword_map.yaml"]
    )

    # ── 런치 인자 ─────────────────────────────────────────
    args = [
        DeclareLaunchArgument("language",    default_value="ko-KR",
                              description="STT 인식 언어"),
        DeclareLaunchArgument("tts_lang",    default_value="ko",
                              description="TTS 출력 언어"),
        DeclareLaunchArgument("mic_index",   default_value="-1",
                              description="마이크 장치 인덱스 (-1: 기본)"),
        DeclareLaunchArgument("vel_scale",   default_value="0.15",
                              description="로봇 최대 속도 스케일"),
        DeclareLaunchArgument("gripper_ip",  default_value="192.168.1.1",
                              description="OnRobot ToolCharger IP 주소"),
        DeclareLaunchArgument("use_gripper", default_value="true",
                              description="그리퍼 활성화 (false: 시뮬레이션 모드)"),
    ]

    # ── 노드 정의 ─────────────────────────────────────────
    nodes = [

        # a. STT 수신 노드
        Node(
            package="stt_robot_demo",
            executable="stt_node",
            name="stt_node",
            output="screen",
            parameters=[{
                "language":         LaunchConfiguration("language"),
                "energy_threshold": 300,
                "pause_threshold":  0.8,
                "phrase_timeout":   3.0,
                "device_index":     LaunchConfiguration("mic_index"),
            }],
        ),

        # b. 자연어 처리 노드
        Node(
            package="stt_robot_demo",
            executable="nlp_node",
            name="nlp_node",
            output="screen",
            parameters=[{
                "keyword_map_path":    keyword_map_path,
                "confidence_threshold": 0.0,
            }],
        ),

        # c. 행동 실행 노드 (MoveIt + OnRobot 그리퍼)
        Node(
            package="stt_robot_demo",
            executable="stt_pick_and_place",
            name="stt_pick_and_place",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                moveit_py_params,
                {
                    "vel_scale":   LaunchConfiguration("vel_scale"),
                    "gripper_ip":  LaunchConfiguration("gripper_ip"),
                    "use_gripper": LaunchConfiguration("use_gripper"),
                },
            ],
        ),

        # d. TTS 안내 노드
        Node(
            package="stt_robot_demo",
            executable="tts_node",
            name="tts_node",
            output="screen",
            parameters=[{
                "language": LaunchConfiguration("tts_lang"),
                "rate":     1.0,
                "volume":   0.9,
                "slow":     False,
            }],
        ),
    ]

    return LaunchDescription(args + nodes)
