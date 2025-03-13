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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    components_config_path = LaunchConfiguration("components_config_path")
    declare_components_config_path_arg = DeclareLaunchArgument(
        "components_config_path",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_description"), "config", "components.yaml"]
        ),
        description=(
            "Additional components configuration file. Components described in this file "
            "are dynamically included in Panther's urdf."
            "Panther options are described here "
            "https://husarion.com/manuals/panther/panther-options/"
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

    rviz_config = LaunchConfiguration("rviz_config")
    declare_rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=PathJoinSubstitution(
            [FindPackageShare("husarion_ugv_description"), "rviz", "robot_model.rviz"]
        ),
        description="RViz configuration file.",
    )

    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    declare_use_joint_state_publisher_arg = DeclareLaunchArgument(
        "use_joint_state_publisher",
        default_value="False",
        description="Flag enabling joint_state_publisher to publish information about joints positions. Should be false when there is a controller that sends this information.",
        choices=["True", "true", "False", "false"],
    )

    use_joint_state_publisher_gui = LaunchConfiguration("use_joint_state_publisher_gui")
    declare_use_joint_state_publisher_gui_arg = DeclareLaunchArgument(
        "use_joint_state_publisher_gui",
        default_value="False",
        description="Flag enabling joint_state_publisher_gui to publish information about joints positions. Should be false when there is a controller that sends this information.",
        choices=["True", "true", "False", "false"],
    )

    wheel_type = LaunchConfiguration("wheel_type")
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
            [FindPackageShare("husarion_ugv_description"), "/launch/load_urdf.launch.py"]
        ),
        launch_arguments={
            "components_config_path": components_config_path,
            "namespace": namespace,
            "robot_model": robot_model,
            "wheel_type": wheel_type,
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("husarion_ugv_description"), "/launch/rviz.launch.py"]
        ),
        launch_arguments={"namespace": namespace, "rviz_config": rviz_config}.items(),
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(use_joint_state_publisher),
    )

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        namespace=namespace,
        emulate_tty=True,
        condition=IfCondition(use_joint_state_publisher_gui),
    )

    actions = [
        declare_components_config_path_arg,
        declare_namespace_arg,
        declare_robot_model_arg,
        declare_rviz_config_arg,
        declare_use_joint_state_publisher_arg,
        declare_use_joint_state_publisher_gui_arg,
        declare_wheel_type_arg,
        load_urdf,
        rviz,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
    ]

    return LaunchDescription(actions)
