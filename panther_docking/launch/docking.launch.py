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
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
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

    apriltag_config_path = LaunchConfiguration("apriltag_config_path")
    declare_apriltag_config_path_arg = DeclareLaunchArgument(
        "apriltag_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_docking"), "config", "apriltag.yaml"]
        ),
        description=("Path to apriltag configuration file. Only used in simulation."),
    )

    namespace = LaunchConfiguration("namespace", default="")
    use_docking = LaunchConfiguration("use_docking", default="True")
    use_sim = LaunchConfiguration("use_sim", default="False")

    log_level = LaunchConfiguration("log_level")
    declare_log_level = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level",
        choices=["debug", "info", "warning", "error"],
    )

    use_wibotic_info = LaunchConfiguration("use_wibotic_info")
    declare_use_wibotic_info_arg = DeclareLaunchArgument(
        "use_wibotic_info",
        default_value="True",
        description="Whether Wibotic information is used",
        choices=[True, False, "True", "False", "true", "false", "1", "0"],
    )

    namespaced_docking_server_config = ReplaceString(
        source_file=docking_server_config_path,
        replacements={
            "<robot_namespace>": namespace,
            "//": "/",
            "<use_wibotic_info_param>": PythonExpression(
                ["'false' if '", use_sim, "' else '", use_wibotic_info, "'"]
            ),
        },
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

    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        parameters=[{"use_sim_time": True}, apriltag_config_path],
        namespace=namespace,
        emulate_tty=True,
        remappings={
            "camera_info": "camera/color/camera_info",
            "image_rect": "camera/color/image_raw",
            "detections": "docking/april_tags",
        }.items(),
        condition=IfCondition(PythonExpression([use_docking, " and ", use_sim])),
    )

    station_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_docking"),
                    "launch",
                    "station.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace}.items(),
    )

    wibotic_connector_can = Node(
        package="wibotic_connector_can",
        executable="wibotic_connector_can",
        namespace=namespace,
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", log_level, "--log-level", "rcl:=INFO"],
        condition=IfCondition(
            PythonExpression(["not ", use_sim, " and ", use_wibotic_info, " and ", use_docking])
        ),
    )

    return LaunchDescription(
        [
            declare_apriltag_config_path_arg,
            declare_docking_server_config_path_arg,
            declare_log_level,
            declare_use_wibotic_info_arg,
            station_launch,
            docking_server_node,
            docking_server_activate_node,
            dock_pose_publisher,
            apriltag_node,
            wibotic_connector_can,
        ]
    )
