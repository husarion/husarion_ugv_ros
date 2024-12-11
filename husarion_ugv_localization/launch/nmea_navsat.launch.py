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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    nmea_params_path = LaunchConfiguration("nmea_params_path")
    declare_nmea_params_path_arg = DeclareLaunchArgument(
        "nmea_params_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_localization"), "config", "nmea_navsat.yaml"]
        ),
        description="Path to the parameter file for the nmea_socket_driver node.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_robot_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Namespace to all launched nodes and use namespace as tf_prefix. This aids in differentiating between multiple robots with the same devices.",
    )

    device_namespace = "gps"
    nmea_driver_node = Node(
        package="nmea_navsat_driver",
        executable="nmea_socket_driver",
        name="nmea_navsat_driver",
        namespace=namespace,
        parameters=[
            {
                "frame_id": device_namespace,
                "tf_prefix": namespace,
            },
            nmea_params_path,
        ],
        remappings=[
            ("fix", [device_namespace, "/fix"]),
            ("time_reference", [device_namespace, "/time_reference"]),
            ("vel", [device_namespace, "/vel"]),
            ("heading", ["_", device_namespace, "/heading"]),
        ],
    )

    return LaunchDescription(
        [
            declare_nmea_params_path_arg,
            declare_robot_namespace_arg,
            nmea_driver_node,
        ]
    )
