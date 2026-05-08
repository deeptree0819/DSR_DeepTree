# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0

# MoveIt2 launch file for Doosan m0609 + OnRobot RG2
#
# 모드별 실행 방법:
#
#   Isaac Sim 연동 (기본):
#     ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py
#
#   실물 로봇:
#     ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
#       mode:=real host:=192.168.1.100
#
#   Doosan 가상 에뮬레이터:
#     ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
#       mode:=virtual
#
#   mock (하드웨어 없이 MoveIt2 단독 테스트):
#     ros2 launch isaac_moveit isaac_moveit_dsr_m0609_rg2.launch.py \
#       mode:=mock

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


# ──────────────────────────────────────────────────────────────────────────────
# 공통 헬퍼
# ──────────────────────────────────────────────────────────────────────────────

def _make_moveit_config(urdf_xacro: str, urdf_mappings: dict, srdf: str, controllers: str):
    return (
        MoveItConfigsBuilder("m0609", package_name="dsr_moveit_config_m0609")
        .robot_description(file_path=urdf_xacro, mappings=urdf_mappings)
        .robot_description_semantic(file_path=srdf)
        .trajectory_execution(file_path=controllers)
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )


def _common_nodes(moveit_config, use_sim_time: bool, rviz_config: str):
    """mode 에 무관하게 항상 필요한 노드들."""
    sim_param = {"use_sim_time": use_sim_time}

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict(), sim_param],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            sim_param,
        ],
    )

    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[sim_param],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description, sim_param],
    )

    return move_group_node, rviz_node, world2robot_tf_node, robot_state_publisher


# ──────────────────────────────────────────────────────────────────────────────
# Isaac Sim 모드
# ──────────────────────────────────────────────────────────────────────────────

def _isaac_mode():
    share = get_package_share_directory("isaac_moveit")

    moveit_config = _make_moveit_config(
        urdf_xacro=os.path.join(share, "config", "dsr_m0609_rg2_isaac.urdf.xacro"),
        urdf_mappings={"ros2_control_hardware_type": "isaac"},
        srdf=os.path.join(share, "config", "dsr_m0609_rg2.srdf"),
        controllers=os.path.join(share, "config", "m0609_rg2_moveit_controllers.yaml"),
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0609"), "launch", "moveit.rviz"
    )
    move_group, rviz, tf, rsp = _common_nodes(moveit_config, use_sim_time=True, rviz_config=rviz_cfg)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(share, "config", "m0609_rg2_controllers.yaml"),
            {"use_sim_time": True},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["dsr_moveit_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    gripper_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["rg2_gripper_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    passthrough = Node(
        package="isaac_moveit",
        executable="joint_state_passthrough.py",
        name="joint_state_passthrough",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            # 수동 모드 종료 시 컨트롤러 hold point 를 현재 Isaac Sim 위치로 동기화
            # → OnImpulseEventCtrl 재개 시 snap-back 없이 새 위치에서 Plan & Execute 가능
            "sync_to_isaac": True,
        }],
    )


    return [
        tf, rsp, ros2_control_node,
        RegisterEventHandler(OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=3.0, actions=[jsb])],
        )),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=arm_ctrl, on_exit=[gripper_ctrl])),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_ctrl, on_exit=[move_group, rviz, passthrough]
        )),
    ]


# ──────────────────────────────────────────────────────────────────────────────
# 실물(real) / 가상(virtual) 모드
# ──────────────────────────────────────────────────────────────────────────────

def _real_mode(context):
    host    = LaunchConfiguration("host").perform(context)
    port    = LaunchConfiguration("port").perform(context)
    rt_host = LaunchConfiguration("rt_host").perform(context)
    mode    = LaunchConfiguration("mode").perform(context)   # "real" | "virtual"

    share = get_package_share_directory("isaac_moveit")

    moveit_config = _make_moveit_config(
        urdf_xacro=os.path.join(share, "config", "dsr_m0609_rg2_real.urdf.xacro"),
        urdf_mappings={"host": host, "port": port, "rt_host": rt_host, "mode": mode},
        srdf=os.path.join(share, "config", "dsr_m0609_rg2.srdf"),
        controllers=os.path.join(share, "config", "m0609_rg2_moveit_controllers.yaml"),
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0609"), "launch", "moveit.rviz"
    )
    move_group, rviz, tf, rsp = _common_nodes(moveit_config, use_sim_time=False, rviz_config=rviz_cfg)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(share, "config", "m0609_rg2_real_controllers.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    # Doosan 에뮬레이터 (real 모드에서도 구동하되 내부적으로 mode 인수 사용)
    run_emulator = Node(
        package="dsr_bringup2",
        executable="run_emulator",
        parameters=[
            {"name": ""},
            {"rate": 100},
            {"standby": 5000},
            {"command": True},
            {"host": host},
            {"port": port},
            {"mode": mode},
            {"model": "m0609"},
            {"gripper": "none"},
            {"mobile": "none"},
            {"rt_host": rt_host},
        ],
        output="screen",
    )

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )
    dsr_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["dsr_controller2", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )
    arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["dsr_moveit_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )
    gripper_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["rg2_gripper_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "120"],
    )

    # Isaac Sim 미러 노드: /joint_states → /isaac_joint_commands
    # Isaac Sim이 실행 중일 때 실물 로봇의 움직임을 시뮬레이션에 반영
    mirror_node = Node(
        package="isaac_moveit",
        executable="joint_state_mirror.py",
        name="joint_state_mirror",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # 수동 모드 동기화 노드 (Physical Inspector ↔ MoveIt2)
    passthrough_node = Node(
        package="isaac_moveit",
        executable="joint_state_passthrough.py",
        name="joint_state_passthrough",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    # 수동 모드 시 Isaac Sim joint state → 실물 컨트롤러 streaming
    # (Isaac Sim Python 3.11 에서 rclpy 를 못 쓰므로 별도 시스템 Python 프로세스로 분리)
    streamer_node = Node(
        package="isaac_moveit",
        executable="isaac_to_real_streamer.py",
        name="isaac_to_real_streamer",
        output="screen",
        parameters=[{"use_sim_time": False}],
    )

    return [
        tf, rsp, run_emulator, ros2_control_node,
        RegisterEventHandler(OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=5.0, actions=[jsb])],
        )),
        RegisterEventHandler(OnProcessExit(target_action=jsb,      on_exit=[dsr_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=dsr_ctrl, on_exit=[arm_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=arm_ctrl, on_exit=[gripper_ctrl])),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_ctrl,
            on_exit=[move_group, rviz, mirror_node, passthrough_node, streamer_node],
        )),
    ]


# ──────────────────────────────────────────────────────────────────────────────
# mock 모드 (하드웨어 없이 MoveIt2 단독 테스트)
# ──────────────────────────────────────────────────────────────────────────────

def _mock_mode():
    share = get_package_share_directory("isaac_moveit")

    moveit_config = _make_moveit_config(
        urdf_xacro=os.path.join(share, "config", "dsr_m0609_rg2_isaac.urdf.xacro"),
        urdf_mappings={"ros2_control_hardware_type": "mock_components"},
        srdf=os.path.join(share, "config", "dsr_m0609_rg2.srdf"),
        controllers=os.path.join(share, "config", "m0609_rg2_moveit_controllers.yaml"),
    )

    rviz_cfg = os.path.join(
        get_package_share_directory("dsr_moveit_config_m0609"), "launch", "moveit.rviz"
    )
    move_group, rviz, tf, rsp = _common_nodes(moveit_config, use_sim_time=False, rviz_config=rviz_cfg)

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            os.path.join(share, "config", "m0609_rg2_controllers.yaml"),
            {"use_sim_time": False},
        ],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    jsb = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    arm_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["dsr_moveit_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )
    gripper_ctrl = Node(
        package="controller_manager", executable="spawner",
        arguments=["rg2_gripper_controller", "-c", "/controller_manager",
                   "--controller-manager-timeout", "60"],
    )

    return [
        tf, rsp, ros2_control_node,
        RegisterEventHandler(OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=3.0, actions=[jsb])],
        )),
        RegisterEventHandler(OnProcessExit(target_action=jsb,      on_exit=[arm_ctrl])),
        RegisterEventHandler(OnProcessExit(target_action=arm_ctrl, on_exit=[gripper_ctrl])),
        RegisterEventHandler(OnProcessExit(
            target_action=gripper_ctrl, on_exit=[move_group, rviz]
        )),
    ]


# ──────────────────────────────────────────────────────────────────────────────
# OpaqueFunction 진입점
# ──────────────────────────────────────────────────────────────────────────────

def _launch_setup(context):
    mode = LaunchConfiguration("mode").perform(context)
    if mode == "isaac":
        return _isaac_mode()
    elif mode in ("real", "virtual"):
        return _real_mode(context)
    elif mode == "mock":
        return _mock_mode()
    else:
        raise ValueError(f"알 수 없는 mode: '{mode}'. 가능한 값: isaac | real | virtual | mock")


# ──────────────────────────────────────────────────────────────────────────────
# LaunchDescription
# ──────────────────────────────────────────────────────────────────────────────

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="isaac",
            description="실행 모드: isaac | real | virtual | mock",
        ),
        DeclareLaunchArgument(
            "host",
            default_value="127.0.0.1",
            description="로봇 IP 주소 (real/virtual 모드에서 사용)",
        ),
        DeclareLaunchArgument(
            "port",
            default_value="12345",
            description="로봇 포트 (real/virtual 모드에서 사용)",
        ),
        DeclareLaunchArgument(
            "rt_host",
            default_value="192.168.137.50",
            description="RT 제어 IP (real 모드에서 사용)",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
