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

import os
from tempfile import NamedTemporaryFile

import imageio
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_apriltag_and_get_path(tag_id):
    from moms_apriltag import TagGenerator2

    tag_generator = TagGenerator2("tag36h11")
    tag_image = tag_generator.generate(tag_id, scale=1000)
    temp_file = NamedTemporaryFile(suffix=f"_tag_{tag_id}.png", delete=False)

    imageio.imwrite(temp_file.name, tag_image)
    return temp_file.name


def spawn_stations(context, *args, **kwargs):
    docking_server_config_path = LaunchConfiguration("docking_server_config_path").perform(context)
    use_docking = LaunchConfiguration("use_docking").perform(context)
    docking_server_config = None

    try:
        with open(os.path.join(docking_server_config_path)) as file:
            docking_server_config = yaml.safe_load(file)
        if not isinstance(docking_server_config, dict) or "/**" not in docking_server_config:
            raise ValueError("Invalid configuration structure")
    except Exception as e:
        print(f"Error loading docking server config: {str(e)}")
        return []

    actions = []

    ros_parameters = docking_server_config["/**"]["ros__parameters"]
    docks_names = ros_parameters["docks"]
    for dock_name in docks_names:
        pose = ros_parameters[dock_name]["pose"]

        spawn_station = Node(
            package="ros_gz_sim",
            executable="create",
            arguments=[
                "-name",
                [dock_name, "_station"],
                "-topic",
                [dock_name, "_station_description"],
                "-x",
                str(pose[0]),
                "-y",
                str(pose[1] - 2.0),  # -2.0 is the offset between world and map
                "-z",
                "0.5",  # station z is not in 0.0
                "-R",
                "1.57",
                "-P",
                "0.0",
                "-Y",
                str(pose[2] - 1.57),
            ],
            emulate_tty=True,
            condition=IfCondition(use_docking),
        )

        actions.append(spawn_station)

    return actions


def generate_launch_description():
    declare_device_namespace = DeclareLaunchArgument(
        "device_namespace",
        default_value="",
        description="Device namespace that will appear before all non absolute topics and TF frames, used for distinguishing multiple cameras on the same robot.",
    )

    declare_docking_server_config_path_arg = DeclareLaunchArgument(
        "docking_server_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_docking"), "config", "panther_docking_server.yaml"]
        ),
        description=("Path to docking server configuration file."),
    )

    declare_use_docking_arg = DeclareLaunchArgument(
        "use_docking",
        default_value="True",
        description="Enable docking server and spawn docking stations in a simulation.",
        choices=["True", "False", "true", "false"],
    )

    return LaunchDescription(
        [
            declare_docking_server_config_path_arg,
            declare_device_namespace,
            declare_use_docking_arg,
            OpaqueFunction(function=spawn_stations),
        ]
    )
