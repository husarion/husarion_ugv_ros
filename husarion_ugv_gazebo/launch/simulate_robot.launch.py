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
    add_world_transform = LaunchConfiguration("add_world_transform")
    declare_add_world_transform_arg = DeclareLaunchArgument(
        "add_world_transform",
        default_value="False",
        description=(
            "Adds a world frame that connects the tf trees of individual robots (useful when running"
            " multiple robots)."
        ),
        choices=["True", "true", "False", "false"],
    )

    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_gazebo"), "config", "battery_plugin.yaml"]
        ),
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_description"), "config", "components.yaml"]
        ),
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in robot's URDF."
            "Available options are described in the manual: "
            "https://husarion.com/manuals/panther/panther-options/"
        ),
    )

    disable_manager = LaunchConfiguration("disable_manager")
    declare_disable_manager_arg = DeclareLaunchArgument(
        "disable_manager",
        default_value="False",
        description="Enable or disable manager_bt_node.",
        choices=["True", "true", "False", "false"],
    )

    gz_bridge_config_path = LaunchConfiguration("gz_bridge_config_path")
    declare_gz_bridge_config_path_arg = DeclareLaunchArgument(
        "gz_bridge_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_gazebo"), "config", "robot_bridge.yaml"]
        ),
        description="Path to the parameter_bridge configuration file.",
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

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model.",
        choices=["lynx", "panther"],
    )

    spawn_robot_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_gazebo"), "launch", "spawn_robot.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": robot_model,
            "log_level": log_level,
        }.items(),
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
            "use_sim": "True",
        }.items(),
        condition=UnlessCondition(PythonExpression(["'", robot_model, "' == 'lynx'"])),
    )

    manager_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_manager"), "launch", "manager.launch.py"]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
        condition=UnlessCondition(
            PythonExpression(["'", robot_model, "' == 'lynx' or ", disable_manager])
        ),
    )

    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_controller"),
                    "launch",
                    "controller.launch.py",
                ]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "publish_robot_state": "False",
            "use_sim": "True",
        }.items(),
    )

    ekf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare("husarion_ugv_localization"),
                    "launch",
                    "localization.launch.py",
                ]
            )
        ),
        launch_arguments={
            "log_level": log_level,
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
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
    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    namespaced_gz_bridge_config_path = ReplaceString(
        source_file=gz_bridge_config_path,
        replacements={"<model_name>": model_name, "<namespace>/": ns},
    )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        parameters=[{"config_file": namespaced_gz_bridge_config_path}],
        namespace=namespace,
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
        ],
        emulate_tty=True,
    )

    child_tf = PythonExpression(["'", namespace, "' + '/odom' if '", namespace, "' else 'odom'"])

    world_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_publisher",
        arguments=[
            "--x",
            LaunchConfiguration("x"),
            "--y",
            LaunchConfiguration("y"),
            "--z",
            LaunchConfiguration("z"),
            "--roll",
            LaunchConfiguration("roll"),
            "--pitch",
            LaunchConfiguration("pitch"),
            "--yaw",
            LaunchConfiguration("yaw"),
            "--frame-id",
            "world",
            "--child-frame-id",
            child_tf,
        ],
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(add_world_transform),
    )

    actions = [
        declare_add_world_transform_arg,
        declare_battery_config_path_arg,
        declare_robot_model_arg,  # robot_model is used by components_config_path
        declare_components_config_path_arg,
        declare_disable_manager_arg,
        declare_gz_bridge_config_path_arg,
        declare_log_level_arg,
        declare_namespace_arg,
        SetUseSimTime(True),
        spawn_robot_launch,
        lights_launch,
        manager_launch,
        controller_launch,
        ekf_launch,
        simulate_components,
        gz_bridge,
        world_transform,
    ]

    return LaunchDescription(actions)
