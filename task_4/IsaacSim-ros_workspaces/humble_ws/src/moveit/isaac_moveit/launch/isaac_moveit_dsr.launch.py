# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# MoveIt2 launch file for Doosan m1013 + Isaac Sim
#
# Prerequisites:
#   1. Build and source the doosan_ros2 workspace so that dsr_moveit_config_m1013
#      and dsr_description2 packages are available in the ament index.
#   2. Source this (Isaac Sim ROS) workspace on top of it.
#
# Usage (Isaac Sim 연동):
#   ros2 launch isaac_moveit isaac_moveit_dsr.launch.py
#
# Usage (Isaac Sim 없이 standalone 테스트):
#   ros2 launch isaac_moveit isaac_moveit_dsr.launch.py \
#     ros2_control_hardware_type:=mock_components use_sim_time:=false

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():

    # --------------------------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------------------------
    ros2_control_hardware_type = DeclareLaunchArgument(
        "ros2_control_hardware_type",
        default_value="isaac",
        description=(
            "ROS2 control hardware interface type -- "
            "possible values: [mock_components, isaac]"
        ),
    )

    use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use Isaac Sim simulation clock if true",
    )

    # --------------------------------------------------------------------------
    # MoveIt configuration (dsr_moveit_config_m1013 as the base package)
    # --------------------------------------------------------------------------
    isaac_moveit_share = get_package_share_directory("isaac_moveit")
    custom_urdf_xacro = os.path.join(
        isaac_moveit_share, "config", "dsr_m1013_isaac.urdf.xacro"
    )

    moveit_config = (
        MoveItConfigsBuilder("m1013", package_name="dsr_moveit_config_m1013")
        .robot_description(
            file_path=custom_urdf_xacro,
            mappings={
                "ros2_control_hardware_type": LaunchConfiguration(
                    "ros2_control_hardware_type"
                )
            },
        )
        .robot_description_semantic(file_path="config/dsr.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # --------------------------------------------------------------------------
    # move_group action server
    # --------------------------------------------------------------------------
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # --------------------------------------------------------------------------
    # RViz — dsr_moveit_controller가 활성화된 후에 실행
    # --------------------------------------------------------------------------
    rviz_config_file = os.path.join(
        get_package_share_directory("dsr_moveit_config_m1013"),
        "launch",
        "moveit.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # --------------------------------------------------------------------------
    # Static TF: world -> base_link
    # Isaac Sim 스테이지에서 로봇 위치에 맞게 xyz/rpy를 조정하세요.
    # --------------------------------------------------------------------------
    world2robot_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher_world_to_robot",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    # --------------------------------------------------------------------------
    # Robot state publisher (TF tree from URDF)
    # --------------------------------------------------------------------------
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    # --------------------------------------------------------------------------
    # ros2_control: controller manager
    # --------------------------------------------------------------------------
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dsr_moveit_config_m1013"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )

    # --------------------------------------------------------------------------
    # Controller spawners — 순차 실행으로 타이밍 문제 방지
    #
    # 실행 순서:
    #   1. ros2_control_node 시작
    #   2. (3초 대기) joint_state_broadcaster 활성화
    #   3. joint_state_broadcaster 완료 후 dsr_moveit_controller 활성화
    #   4. dsr_moveit_controller 완료 후 move_group + RViz 실행
    # --------------------------------------------------------------------------
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    dsr_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "dsr_moveit_controller",
            "-c", "/controller_manager",
            "--controller-manager-timeout", "60",
        ],
    )

    # ros2_control_node 시작 후 3초 대기 → joint_state_broadcaster 실행
    delay_jsb_after_control = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[joint_state_broadcaster_spawner],
                )
            ],
        )
    )

    # joint_state_broadcaster 완료 후 → dsr_moveit_controller 실행
    delay_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[dsr_arm_controller_spawner],
        )
    )

    # dsr_moveit_controller 완료 후 → move_group + RViz 실행
    delay_moveit_after_controller = RegisterEventHandler(
        OnProcessExit(
            target_action=dsr_arm_controller_spawner,
            on_exit=[move_group_node, rviz_node],
        )
    )

    # --------------------------------------------------------------------------
    # Launch description
    # --------------------------------------------------------------------------
    return LaunchDescription(
        [
            ros2_control_hardware_type,
            use_sim_time,
            world2robot_tf_node,
            robot_state_publisher,
            ros2_control_node,
            delay_jsb_after_control,
            delay_arm_after_jsb,
            delay_moveit_after_controller,
        ]
    )
