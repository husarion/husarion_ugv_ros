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
from launch.actions import DeclareLaunchArgument  # , IncludeLaunchDescription
from launch.conditions import IfCondition  # , UnlessCondition

# from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    docking_server_config_path = LaunchConfiguration("docking_server_config_path")
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
        description="Enable docking server.",
        choices=["True", "False", "true", "false"],
    )

    namespace = LaunchConfiguration("namespace")
    use_docking = LaunchConfiguration("use_docking")
    use_sim = LaunchConfiguration("use_sim")

    log_level = LaunchConfiguration("log_level")
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
        choices=["debug", "info", "warning", "error"],
    )

    namespaced_docking_server_config = ReplaceString(
        source_file=docking_server_config_path,
        replacements={"<robot_namespace>": namespace, "//": "/"},
    )

    docking_server_node = Node(
        package="opennav_docking",
        executable="opennav_docking",
        namespace=namespace,
        parameters=[
            namespaced_docking_server_config,
            {"use_sim_time": use_sim},
        ],
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        remappings=[("~/transition_event", "~/_transition_event")],
        emulate_tty=True,
        condition=IfCondition(use_docking),
    )

    docking_server_activate_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="nav2_docking_lifecycle_manager",
        namespace=namespace,
        parameters=[
            {
                "autostart": True,
                "node_names": [
                    "docking_server",
                ],
                "use_sim_time": use_sim,
            },
        ],
        condition=IfCondition(use_docking),
    )

    dock_pose_publisher = Node(
        package="panther_docking",
        executable="dock_pose_publisher",
        parameters=[
            namespaced_docking_server_config,
            {"use_sim_time": use_sim},
        ],
        name="dock_pose_publisher",
        namespace=namespace,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        condition=IfCondition(use_docking),
    )

    # FIXME: This launch does not work with the simulation. It can be caused by different versions of opencv
    # station_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution(
    #             [
    #                 FindPackageShare("panther_docking"),
    #                 "launch",
    #                 "station.launch.py",
    #             ]
    #         ),
    #     ),
    #     launch_arguments={"namespace": namespace}.items(),
    #     condition=UnlessCondition(use_sim),
    # )

    return LaunchDescription(
        [
            declare_use_docking_arg,
            declare_docking_server_config_path_arg,
            declare_log_level,
            # station_launch,
            docking_server_node,
            docking_server_activate_node,
            dock_pose_publisher,
        ]
    )
