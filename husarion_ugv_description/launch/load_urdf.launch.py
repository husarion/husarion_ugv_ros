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

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    EnvironmentVariable,
    FindExecutable,
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

    description_pkg = FindPackageShare("husarion_ugv_description")
    description_common_dir = PythonExpression(
        [
            "'",
            common_dir_path,
            "/husarion_ugv_description",
            "' if '",
            common_dir_path,
            "' else '",
            description_pkg,
            "'",
        ]
    )

    battery_config_path = LaunchConfiguration("battery_config_path")
    declare_battery_config_path_arg = DeclareLaunchArgument(
        "battery_config_path",
        description=(
            "Path to the Ignition LinearBatteryPlugin configuration file. "
            "This configuration is intended for use in simulations only."
        ),
        default_value="",
    )

    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution([description_common_dir, "config", "components.yaml"]),
        description=(
            "Specify file which contains components. These components will be included in URDF."
            "Available options can be found in manuals: https://husarion.com/manuals"
        ),
    )

    wheel_type = LaunchConfiguration("wheel_type")
    controller_config_path = LaunchConfiguration("controller_config_path")
    declare_controller_config_path_arg = DeclareLaunchArgument(
        "controller_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_controller"),
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
        description="Specify robot model",
        choices=["lynx", "panther"],
    )

    use_sim = LaunchConfiguration("use_sim")
    declare_use_sim_arg = DeclareLaunchArgument(
        "use_sim",
        default_value="False",
        description="Whether simulation is used.",
        choices=["True", "true", "False", "false"],
    )

    wheel_config_path = LaunchConfiguration("wheel_config_path")
    declare_wheel_config_path_arg = DeclareLaunchArgument(
        "wheel_config_path",
        default_value=PathJoinSubstitution(
            [
                FindPackageShare("husarion_ugv_description"),
                "config",
                PythonExpression(["'", wheel_type, ".yaml'"]),
            ]
        ),
        description=(
            "Path to wheel configuration file. By default, it is located in "
            "'husarion_ugv_description/config/{wheel_type}.yaml'. You can also specify the path "
            "to your custom wheel configuration file here. "
        ),
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

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    ns_controller_config_path = ReplaceString(controller_config_path, {"<namespace>/": ns})

    # Get URDF via xacro
    imu_pos_x = os.environ.get("ROBOT_IMU_LOCALIZATION_X", "0.168")
    imu_pos_y = os.environ.get("ROBOT_IMU_LOCALIZATION_Y", "0.028")
    imu_pos_z = os.environ.get("ROBOT_IMU_LOCALIZATION_Z", "0.083")
    imu_rot_r = os.environ.get("ROBOT_IMU_ORIENTATION_R", "3.14")
    imu_rot_p = os.environ.get("ROBOT_IMU_ORIENTATION_P", "-1.57")
    imu_rot_y = os.environ.get("ROBOT_IMU_ORIENTATION_Y", "0.0")
    urdf_file = PythonExpression(["'", robot_model, ".urdf.xacro'"])
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("husarion_ugv_description"), "urdf", urdf_file]
            ),
            " use_sim:=",
            use_sim,
            " wheel_config_file:=",
            wheel_config_path,
            " controller_config_file:=",
            ns_controller_config_path,
            " battery_config_file:=",
            battery_config_path,
            " imu_xyz:=",
            f"'{imu_pos_x} {imu_pos_y} {imu_pos_z}'",
            " imu_rpy:=",
            f"'{imu_rot_r} {imu_rot_p} {imu_rot_y}'",
            " namespace:=",
            namespace,
            " components_config_path:=",
            components_config_path,
        ]
    )

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_description_content},
            {"frame_prefix": namespace_ext},
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    actions = [
        declare_common_dir_path_arg,
        declare_battery_config_path_arg,
        declare_components_config_path_arg,
        declare_robot_model_arg,  # robot_model is used by wheel_type
        declare_wheel_type_arg,  # wheel_type is used by controller_config_path
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_wheel_config_path_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        robot_state_pub_node,
    ]

    return LaunchDescription(actions)
