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

import imageio
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_apriltag_and_get_path(tag_id):
    from moms_apriltag import TagGenerator2

    tag_generator = TagGenerator2("tag36h11")
    tag_image = tag_generator.generate(tag_id, scale=1000)

    path = f"/tmp/tag_{tag_id}.png"

    imageio.imwrite(path, tag_image)
    return path


def launch_setup(context, *args, **kwargs):
    namespace = LaunchConfiguration("namespace").perform(context)
    apriltag_id = int(LaunchConfiguration("apriltag_id").perform(context))
    apriltag_size = LaunchConfiguration("apriltag_size").perform(context)
    use_docking = LaunchConfiguration("use_docking").perform(context)

    apriltag_image_path = generate_apriltag_and_get_path(apriltag_id)

    station_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_components_description"),
                    "urdf",
                    "wibotic_station.urdf.xacro",
                ]
            ),
            " device_namespace:=",
            "main",
            " apriltag_image_path:=",
            apriltag_image_path,
            " apriltag_size:=",
            apriltag_size,
        ]
    )

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    station_description = {
        "robot_description": ParameterValue(station_description_content, value_type=str)
    }

    station_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="wibotic_station_state_publisher",
        parameters=[
            station_description,
            {"frame_prefix": namespace_ext},
        ],
        remappings=[("robot_description", "station_description")],
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(use_docking),
    )

    return [station_state_pub_node]


def generate_launch_description():
    declare_use_docking_arg = DeclareLaunchArgument(
        "use_docking",
        default_value="True",
        description="Enable docking server.",
        choices=["True", "False", "true", "false"],
    )

    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    declare_apriltag_id = DeclareLaunchArgument(
        "apriltag_id",
        default_value="0",
        description="ID of a generated apriltag on the station",
    )

    declare_apriltag_size = DeclareLaunchArgument(
        "apriltag_size",
        default_value="0.06",
        description="Size in meters of a generated apriltag on the station",
    )

    return LaunchDescription(
        [
            declare_use_docking_arg,
            declare_namespace_arg,
            declare_apriltag_id,
            declare_apriltag_size,
            OpaqueFunction(function=launch_setup),
        ]
    )
