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


from husarion_ugv_utils.messages import welcome_msg
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, SetUseSimTime
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

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

    x = LaunchConfiguration("x")
    declare_x_arg = DeclareLaunchArgument(
        "x", default_value="0.0", description="Initial robot position in the global 'x' axis."
    )

    y = LaunchConfiguration("y")
    declare_y_arg = DeclareLaunchArgument(
        "y", default_value="-2.0", description="Initial robot position in the global 'y' axis."
    )

    z = LaunchConfiguration("z")
    declare_z_arg = DeclareLaunchArgument(
        "z", default_value="0.0", description="Initial robot position in the global 'z' axis."
    )

    roll = LaunchConfiguration("roll")
    declare_roll_arg = DeclareLaunchArgument(
        "roll", default_value="0.0", description="Initial robot 'roll' orientation."
    )

    pitch = LaunchConfiguration("pitch")
    declare_pitch_arg = DeclareLaunchArgument(
        "pitch", default_value="0.0", description="Initial robot 'pitch' orientation."
    )

    yaw = LaunchConfiguration("yaw")
    declare_yaw_arg = DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Initial robot 'yaw' orientation."
    )

    log_stats = {
        "Robot namespace": namespace,
        "Initial pose": ["(", x, ", ", y, ", ", z, ", ", roll, ", ", pitch, ", ", yaw, ")"],
    }
    welcome_info = welcome_msg(robot_model, "----", "simulation", log_stats)

    robot_description_pkg = PythonExpression(["'", robot_model, "_description'"])
    load_urdf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare(robot_description_pkg), "launch", "load_urdf.launch.py"]
            )
        ),
        launch_arguments={
            "add_wheel_joints": LaunchConfiguration("add_wheel_joints", default="True"),
            "namespace": namespace,
            "use_sim": "True",
        }.items(),
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            namespace,
            "-topic",
            "robot_description",
            "-x",
            x,
            "-y",
            y,
            "-z",
            z,
            "-R",
            roll,
            "-P",
            pitch,
            "-Y",
            yaw,
        ],
        namespace=namespace,
        emulate_tty=True,
    )

    actions = [
        declare_namespace_arg,
        declare_robot_model_arg,
        declare_x_arg,
        declare_y_arg,
        declare_z_arg,
        declare_roll_arg,
        declare_pitch_arg,
        declare_yaw_arg,
        SetUseSimTime(True),
        welcome_info,
        load_urdf,
        spawn_robot,
    ]

    return LaunchDescription(actions)
