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
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
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
        description="Add namespace to all launched nodes",
    )

    system_monitor_config_path = LaunchConfiguration("system_monitor_config_path")
    declare_system_monitor_config_path_arg = DeclareLaunchArgument(
        "system_monitor_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_diagnostics"),
                "config",
                "system_monitor.yaml",
            ]
        ),
        description="Specify the path to the system monitor configuration file.",
    )

    system_monitor_node = Node(
        package="husarion_ugv_diagnostics",
        executable="system_monitor_node",
        name="system_monitor",
        parameters=[system_monitor_config_path],
        namespace=namespace,
        remappings=[("/diagnostics", "diagnostics")],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
    )

    actions = [
        declare_log_level_arg,
        declare_namespace_arg,
        declare_system_monitor_config_path_arg,
        system_monitor_node,
    ]

    return LaunchDescription(actions)
