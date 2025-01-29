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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():

    gz_gui = LaunchConfiguration("gz_gui")
    declare_gz_gui = DeclareLaunchArgument(
        "gz_gui",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_gazebo"), "config", "teleop_with_estop.config"]
        ),
        description="Run simulation with specific GUI layout.",
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

    use_rviz = LaunchConfiguration("use_rviz")
    declare_use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="True",
        description="Run RViz simultaneously.",
        choices=["True", "true", "False", "false"],
    )

    namespaced_gz_gui = ReplaceString(
        source_file=gz_gui,
        replacements={"{namespace}": namespace},
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_gz_worlds"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_gui": namespaced_gz_gui, "gz_log_level": "1"}.items(),
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_description"),
                    "launch",
                    "rviz.launch.py",
                ]
            )
        ),
        condition=IfCondition(use_rviz),
    )

    simulate_robots = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_gazebo"),
                    "launch",
                    "simulate_robot.launch.py",
                ]
            )
        ),
        launch_arguments={"log_level": log_level}.items(),
    )

    actions = [
        declare_gz_gui,
        declare_log_level_arg,
        declare_namespace_arg,
        declare_use_rviz_arg,
        # Sets use_sim_time for all nodes started below (doesn't work for nodes started from ignition gazebo)
        SetUseSimTime(True),
        gz_sim,
        rviz_launch,
        simulate_robots,
    ]

    return LaunchDescription(actions)
