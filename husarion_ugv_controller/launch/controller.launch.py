#!/usr/bin/env python3

# Copyright 2020 ros2_control Development Team
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
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString


def generate_launch_description():
    common_dir_path = LaunchConfiguration("common_dir_path")
    declare_common_dir_path_arg = DeclareLaunchArgument(
        "common_dir_path",
        default_value="",
        description="Path to the common configuration directory.",
    )
    husarion_ugv_controller_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/husarion_ugv_controller' if '",
            common_dir_path,
            "' else '",
            FindPackageShare("husarion_ugv_controller"),
            "'",
        ]
    )

    robot_model = LaunchConfiguration("robot_model")
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model",
        choices=["lynx", "panther"],
    )

    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                husarion_ugv_controller_common_dir,
                "config",
                PythonExpression(["'", wheel_type, "_controller.yaml'"]),
            ]
        ),
        description=(
            "Path to controller configuration file. By default, it is located in"
            " 'husarion_ugv_controller/config/{wheel_type}_controller.yaml'. You can also specify"
            " the path to your custom controller configuration file here. "
        ),
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

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used",
        choices=["True", "true", "False", "false"],
    )

    default_wheel_type = {"lynx": "WH05", "panther": "WH01"}
    declare_wheel_type_arg = DeclareLaunchArgument(
        "wheel_type",
        default_value=PythonExpression([f"{default_wheel_type}['", robot_model, "']"]),
        description=(
            "Specify the wheel type. If the selected wheel type is not 'custom', "
            "the 'wheel_config_path' and 'controller_config_path' arguments will be "
            "automatically adjusted and can be omitted."
        ),
        choices=["WH01", "WH02", "WH04", "WH05", "custom"],
    )

    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_description"), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "namespace": namespace,
            "robot_model": robot_model,
            "log_level": log_level,
        }.items(),
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config_path = ReplaceString(controller_config_path, {"<namespace>/": ns})

    joint_state_broadcaster_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.joint_state_broadcaster' if '",
            namespace,
            "' else 'joint_state_broadcaster'",
        ]
    )
    controller_manager_log_unit = PythonExpression(
        [
            "'",
            namespace,
            "' + '.controller_manager' if '",
            namespace,
            "' else 'controller_manager'",
        ]
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ns_controller_config_path],
        namespace=namespace,
        remappings=[
            ("/diagnostics", "diagnostics"),
            ("drive_controller/cmd_vel", "cmd_vel"),
            ("drive_controller/odom", "odometry/wheels"),
            ("drive_controller/transition_event", "_drive_controller/transition_event"),
            ("imu_broadcaster/imu", "imu/data"),
            ("imu_broadcaster/transition_event", "_imu_broadcaster/transition_event"),
            (
                "joint_state_broadcaster/transition_event",
                "_joint_state_broadcaster/transition_event",
            ),
        ],
        arguments=[
            "--ros-args",
            "--log-level",
            log_level,
            "--log-level",
            limit_log_level_to_info("rcl", log_level),
            "--log-level",
            limit_log_level_to_info("pluginlib.ClassLoader", log_level),
            "--log-level",
            limit_log_level_to_info(joint_state_broadcaster_log_unit, log_level),
            "--log-level",
            limit_log_level_to_info(controller_manager_log_unit, log_level),
        ],
        condition=UnlessCondition(use_sim),
        emulate_tty=True,
        on_exit=Shutdown(),
    )

    spawner_common_args = [
        "--controller-manager",
        "controller_manager",
        "--controller-manager-timeout",
        "10",
        "--ros-args",
        "--log-level",
        log_level,
        "--log-level",
        limit_log_level_to_info("rcl", log_level),
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", *spawner_common_args],
        namespace=namespace,
        emulate_tty=True,
    )

    drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["drive_controller", *spawner_common_args],
        namespace=namespace,
        emulate_tty=True,
    )

    imu_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["imu_broadcaster", *spawner_common_args],
        namespace=namespace,
        emulate_tty=True,
    )

    # Launch spawner one after another
    # when spawning without delay ros2_control_node sometimes crashed
    delay_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[drive_controller_spawner],
        ),
    )

    delay_imu_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=drive_controller_spawner,
            on_exit=[imu_broadcaster_spawner],
        ),
    )

    spawners = GroupAction(
        actions=[
            joint_state_broadcaster_spawner,
            delay_drive_controller_spawner,
            delay_imu_broadcaster_spawner,
        ]
    )

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,  # robot_model is used by wheel_type
        declare_wheel_type_arg,  # wheel_type is used by controller_config_path
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_log_level_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        load_urdf,
        control_node,
        spawners,
    ]

    return LaunchDescription(actions)
