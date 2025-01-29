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


from husarion_ugv_utils.messages import welcome_msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )

    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "true", "False", "false"],
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

    robot_model_name = EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther")
    robot_serial_no = EnvironmentVariable(name="ROBOT_SERIAL_NO", default_value="----")
    robot_version = EnvironmentVariable(name="ROBOT_VERSION", default_value="1.0")
    welcome_info = welcome_msg(robot_model_name, robot_serial_no, robot_version)

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_controller"), "launch", "controller.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "common_dir_path": common_dir_path,
        }.items(),
    )

    system_monitor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_diagnostics"),
                    "launch",
                    "system_monitor.launch.py",
                ]
            ),
        ),
        launch_arguments={"log_level": log_level, "namespace": namespace}.items(),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "common_dir_path": common_dir_path,
        }.items(),
    )

    battery_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_battery"), "launch", "battery.launch.py"]
            ),
        ),
        launch_arguments={"log_level": log_level, "namespace": namespace}.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_localization"), "launch", "localization.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "common_dir_path": common_dir_path,
        }.items(),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_manager"), "launch", "manager.launch.py"]
            )
        ),
        condition=UnlessCondition(disable_manager),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "common_dir_path": common_dir_path,
        }.items(),
    )

    delayed_action = TimerAction(
        period=10.0,
        actions=[
            battery_launch,
            lights_launch,
            manager_launch,
            ekf_launch,
        ],
    )

    actions = [
        declare_common_dir_path_arg,
        declare_disable_manager_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        welcome_info,
        controller_launch,
        system_monitor_launch,
        delayed_action,
    ]

    return LaunchDescription(actions)
