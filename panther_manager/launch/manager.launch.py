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
from launch.conditions import IfCondition
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    panther_version = EnvironmentVariable(name="PANTHER_ROBOT_VERSION", default_value="1.0")
    panther_manager_dir = FindPackageShare("panther_manager")

    docking_bt_project_path = LaunchConfiguration("docking_bt_project_path")
    declare_docking_bt_project_path_arg = DeclareLaunchArgument(
        "docking_bt_project_path",
        default_value=PathJoinSubstitution(
            [panther_manager_dir, "behavior_trees", "DockingBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for docking management.",
    )

    lights_bt_project_path = LaunchConfiguration("lights_bt_project_path")
    declare_lights_bt_project_path_arg = DeclareLaunchArgument(
        "lights_bt_project_path",
        default_value=PathJoinSubstitution(
            [panther_manager_dir, "behavior_trees", "PantherLightsBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for lights management.",
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
            [panther_manager_dir, "behavior_trees", "PantherSafetyBT.btproj"]
        ),
        description="Path to BehaviorTree project file, responsible for safety and shutdown management.",
        condition=IfCondition(PythonExpression([panther_version, ">=", "1.2"])),
    )

    shutdown_hosts_config_path = LaunchConfiguration("shutdown_hosts_config_path")
    declare_shutdown_hosts_config_path_arg = DeclareLaunchArgument(
        "shutdown_hosts_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("panther_manager"),
                "config",
                "shutdown_hosts_config.yaml",
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

    docking_manager_node = Node(
        package="panther_manager",
        executable="docking_manager_node",
        name="docking_manager",
        parameters=[
            PathJoinSubstitution([panther_manager_dir, "config", "docking_manager.yaml"]),
            {"bt_project_path": docking_bt_project_path},
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    lights_manager_node = Node(
        package="panther_manager",
        executable="lights_manager_node",
        name="lights_manager",
        parameters=[
            PathJoinSubstitution([panther_manager_dir, "config", "lights_manager.yaml"]),
            {"bt_project_path": lights_bt_project_path},
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    safety_manager_node = Node(
        package="panther_manager",
        executable="safety_manager_node",
        name="safety_manager",
        parameters=[
            PathJoinSubstitution([panther_manager_dir, "config", "safety_manager.yaml"]),
            {
                "bt_project_path": safety_bt_project_path,
                "shutdown_hosts_path": shutdown_hosts_config_path,
            },
        ],
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(PythonExpression([panther_version, ">=", "1.2 and not ", use_sim])),
    )

    actions = [
        declare_docking_bt_project_path_arg,
        declare_lights_bt_project_path_arg,
        declare_safety_bt_project_path_arg,
        declare_namespace_arg,
        declare_shutdown_hosts_config_path_arg,
        declare_use_sim_arg,
        docking_manager_node,
        lights_manager_node,
        safety_manager_node,
    ]

    return LaunchDescription(actions)
