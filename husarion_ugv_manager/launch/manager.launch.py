#!/usr/bin/env python3

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from husarion_ugv_utils.logging import limit_log_level_to_info
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    husarion_ugv_manager_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/husarion_ugv_manager' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("husarion_ugv_manager"),
            "'",
        ]
    )

    husarion_ugv_manager_pkg = FindPackageShare("husarion_ugv_manager")

    lights_bt_project_path = LaunchConfiguration("lights_bt_project_path")
    declare_lights_bt_project_path_arg = DeclareLaunchArgument(
        "lights_bt_project_path",
        default_value=PathJoinSubstitution(
            [husarion_ugv_manager_common_dir, "behavior_trees", "LightsBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for lights management.",
    )

    log_level = LaunchConfiguration("log_level")
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "FATAL"],
        description="Logging level",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    safety_bt_project_path = LaunchConfiguration("safety_bt_project_path")
    declare_safety_bt_project_path_arg = DeclareLaunchArgument(
        "safety_bt_project_path",
        default_value=PathJoinSubstitution(
            [husarion_ugv_manager_pkg, "behavior_trees", "SafetyBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for safety and shutdown management.",
    )

    shutdown_hosts_config_path = LaunchConfiguration("shutdown_hosts_config_path")
    declare_shutdown_hosts_config_path_arg = DeclareLaunchArgument(
        "shutdown_hosts_config_path",
        default_value=PathJoinSubstitution(
            [
                husarion_ugv_manager_common_dir,
                "config",
                "shutdown_hosts.yaml",
            ]
        ),
        description="Path to file with list of hosts to request shutdown.",
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
    )

    lights_manager_node = Node(
        package="husarion_ugv_manager",
        executable="lights_manager_node",
        name="lights_manager",
        parameters=[
            PathJoinSubstitution([husarion_ugv_manager_pkg, "config", "lights_manager.yaml"]),
            {
                "bt_project_path": lights_bt_project_path,
            },
        ],
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
    )

    safety_manager_node = Node(
        package="husarion_ugv_manager",
        executable="safety_manager_node",
        name="safety_manager",
        parameters=[
            PathJoinSubstitution([husarion_ugv_manager_pkg, "config", "safety_manager.yaml"]),
            {
                "bt_project_path": safety_bt_project_path,
                "shutdown_hosts_path": shutdown_hosts_config_path,
            },
        ],
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
        condition=UnlessCondition(use_sim),
    )

    actions = [
        declare_common_dir_path_arg,
        declare_log_level_arg,
        declare_lights_bt_project_path_arg,
        declare_safety_bt_project_path_arg,
        declare_namespace_arg,
        declare_shutdown_hosts_config_path_arg,
        declare_use_sim_arg,
        lights_manager_node,
        safety_manager_node,
    ]

    return LaunchDescription(actions)
