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
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():

    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "battery_plugin_config.yaml"]
        ),
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_description"), "config", "components.yaml"]
        ),
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in Panther's urdf."
            "Panther options are described here "
            "https://husarion.com/manuals/panther/panther-options/"
        ),
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("panther_gazebo"), "config", "gz_bridge.yaml"]
        ),
        description="Path to the parameter_bridge configuration file.",
    )

    namespace = LaunchConfiguration("namespace")
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )

    use_ekf = LaunchConfiguration("use_ekf")
    declare_use_ekf_arg = DeclareLaunchArgument(
        "use_ekf",
        default_value="True",
        description="Enable or disable EKF.",
        choices=["True", "true", "False", "false"],
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_gazebo"), "launch", "spawn_robot.launch.py"]
            )
        ),
        launch_arguments={
            "add_wheel_joints": "False",
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
    )

    lights_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_lights"), "launch", "lights.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("panther_manager"), "launch", "manager.launch.py"]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    gz_led_strip_config = PathJoinSubstitution(
        [FindPackageShare("panther_gazebo"), "config", "led_strips.yaml"]
    )

    gz_led_strip_config = ReplaceString(
        source_file=gz_led_strip_config,
        replacements={"parent_link: panther": ["parent_link: ", namespace]},
        condition=UnlessCondition(PythonExpression(["'", namespace, "' == ''"])),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "publish_robot_state": "False",
            "use_sim": "True",
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_localization"),
                    "launch",
                    "localization.launch.py",
                ]
            )
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
        condition=IfCondition(use_ekf),
    )

    simulate_components = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("ros_components_description"),
                    "launch",
                    "gz_components.launch.py",
                ]
            )
        ),
        launch_arguments={
            "components_config_path": components_config_path,
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
    )

    model_name = PythonExpression(["'", namespace, "' if '", namespace, "' else 'panther'"])

    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={"<model_name>": model_name, "<namespace>": namespace, "//": "/"},
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="panther_base_gz_bridge",
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=namespace,
        emulate_tty=True,
    )

    docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_docking"),
                    "launch",
                    "docking.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    apriltag_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("panther_gazebo"),
                    "launch",
                    "apriltag.launch.py",
                ]
            ),
        ),
        launch_arguments={"namespace": namespace, "use_sim": "True"}.items(),
    )

    return LaunchDescription(
        [
            declare_battery_config_path_arg,
            declare_components_config_path_arg,
            declare_gz_bridge_config_path_arg,
            declare_namespace_arg,
            declare_use_ekf_arg,
            SetUseSimTime(True),
            spawn_robot_launch,
            lights_launch,
            manager_launch,
            controller_launch,
            ekf_launch,
            simulate_components,
            gz_bridge,
            docking_launch,
            apriltag_launch,
        ]
    )
