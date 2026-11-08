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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, Shutdown
from launch.conditions import UnlessCondition
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

    use_madgwick_filter = LaunchConfiguration("use_madgwick_filter")
    declare_use_madgwick_filter_arg = DeclareLaunchArgument(
        "use_madgwick_filter",
        default_value="False",
        description="Determine orientation from IMU",
        choices=["True", "true", "False", "false"],
    )

    configure_thread_priority = LaunchConfiguration("configure_thread_priority")
    declare_configure_thread_priority_arg = DeclareLaunchArgument(
        "configure_thread_priority",
        default_value="0",
        description=(
            "SCHED_FIFO priority of the thread that configures the hardware. Threads it creates"
            " afterwards inherit it. 0 leaves it unchanged, so only the four RT threads run RT"
        ),
    )

    canopen_thread_priority = LaunchConfiguration("canopen_thread_priority")
    declare_canopen_thread_priority_arg = DeclareLaunchArgument(
        "canopen_thread_priority",
        default_value="70",
        description=(
            "SCHED_FIFO priority of the CANopen communication thread, 1-98. Keep it above the"
            " control loop"
        ),
    )

    rpdo_dispatch_priority = LaunchConfiguration("rpdo_dispatch_priority")
    declare_rpdo_dispatch_priority_arg = DeclareLaunchArgument(
        "rpdo_dispatch_priority",
        default_value="65",
        description=(
            "SCHED_FIFO priority of the RPDO dispatch threads. 0 sets none, so they keep the"
            " priority they inherit"
        ),
    )

    gpio_watchdog_priority = LaunchConfiguration("gpio_watchdog_priority")
    declare_gpio_watchdog_priority_arg = DeclareLaunchArgument(
        "gpio_watchdog_priority",
        default_value="60",
        description=(
            "SCHED_FIFO priority of the safety watchdog thread, 1-98. Keep it above the control loop"
        ),
    )

    gpio_monitor_priority = LaunchConfiguration("gpio_monitor_priority")
    declare_gpio_monitor_priority_arg = DeclareLaunchArgument(
        "gpio_monitor_priority",
        default_value="60",
        description=(
            "SCHED_FIFO priority of the GPIO monitor thread. 0 sets none, so it keeps the priority"
            " it inherits"
        ),
    )

    canopen_thread_cpu = LaunchConfiguration("canopen_thread_cpu")
    declare_canopen_thread_cpu_arg = DeclareLaunchArgument(
        "canopen_thread_cpu",
        default_value="-1",
        description="CPU the CANopen communication thread pins itself to. -1 leaves it unpinned",
    )

    rpdo_dispatch_cpu = LaunchConfiguration("rpdo_dispatch_cpu")
    declare_rpdo_dispatch_cpu_arg = DeclareLaunchArgument(
        "rpdo_dispatch_cpu",
        default_value="-1",
        description="CPU the RPDO dispatch threads pin themselves to. -1 leaves them unpinned",
    )

    gpio_watchdog_cpu = LaunchConfiguration("gpio_watchdog_cpu")
    declare_gpio_watchdog_cpu_arg = DeclareLaunchArgument(
        "gpio_watchdog_cpu",
        default_value="-1",
        description="CPU the safety watchdog thread pins itself to. -1 leaves it unpinned",
    )

    gpio_monitor_cpu = LaunchConfiguration("gpio_monitor_cpu")
    declare_gpio_monitor_cpu_arg = DeclareLaunchArgument(
        "gpio_monitor_cpu",
        default_value="-1",
        description="CPU the GPIO monitor thread pins itself to. -1 leaves it unpinned",
    )

    pdo_motor_states_timeout_ms = LaunchConfiguration("pdo_motor_states_timeout_ms")
    declare_pdo_motor_states_timeout_ms_arg = DeclareLaunchArgument(
        "pdo_motor_states_timeout_ms",
        default_value="15",
        description="Allowed age of the motor state PDOs in ms",
    )

    pdo_driver_state_timeout_ms = LaunchConfiguration("pdo_driver_state_timeout_ms")
    declare_pdo_driver_state_timeout_ms_arg = DeclareLaunchArgument(
        "pdo_driver_state_timeout_ms",
        default_value="75",
        description="Allowed age of the driver state PDOs in ms",
    )

    limit_tpdo_rates = LaunchConfiguration("limit_tpdo_rates")
    declare_limit_tpdo_rates_arg = DeclareLaunchArgument(
        "limit_tpdo_rates",
        default_value="False",
        description=(
            "Write TPDO inhibit and event timers into the motor controllers at boot. It caps the"
            " battery and temperature TPDO at about 25 Hz and gives the motor state TPDOs a 5 ms"
            " timer"
        ),
        choices=["True", "true", "False", "false"],
    )

    canopen_resync_watchdog = LaunchConfiguration("canopen_resync_watchdog")
    declare_canopen_resync_watchdog_arg = DeclareLaunchArgument(
        "canopen_resync_watchdog",
        default_value="False",
        description=(
            "Exit the driver with code 74 after canopen_resync_watchdog_timeout_s of continuous"
            " PDO timeout or 3 lost SDO confirmations within a minute, so that a restart policy"
            " initializes the CANopen master again"
        ),
        choices=["True", "true", "False", "false"],
    )

    canopen_resync_watchdog_timeout_s = LaunchConfiguration("canopen_resync_watchdog_timeout_s")
    declare_canopen_resync_watchdog_timeout_s_arg = DeclareLaunchArgument(
        "canopen_resync_watchdog_timeout_s",
        default_value="30",
        description="Seconds of continuous PDO timeout after which canopen_resync_watchdog exits",
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
            "use_madgwick_filter": use_madgwick_filter,
            "configure_thread_priority": configure_thread_priority,
            "canopen_thread_priority": canopen_thread_priority,
            "rpdo_dispatch_priority": rpdo_dispatch_priority,
            "gpio_watchdog_priority": gpio_watchdog_priority,
            "gpio_monitor_priority": gpio_monitor_priority,
            "canopen_thread_cpu": canopen_thread_cpu,
            "rpdo_dispatch_cpu": rpdo_dispatch_cpu,
            "gpio_watchdog_cpu": gpio_watchdog_cpu,
            "gpio_monitor_cpu": gpio_monitor_cpu,
            "pdo_motor_states_timeout_ms": pdo_motor_states_timeout_ms,
            "pdo_driver_state_timeout_ms": pdo_driver_state_timeout_ms,
            "limit_tpdo_rates": limit_tpdo_rates,
            "canopen_resync_watchdog": canopen_resync_watchdog,
            "canopen_resync_watchdog_timeout_s": canopen_resync_watchdog_timeout_s,
        }.items(),
    )

    ns = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])

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

    orientation_covariance = PythonExpression(
        [
            "[1.8e-3, 0.0, 0.0, 0.0, 1.8e-3, 0.0, 0.0, 0.0, 1.8e-3] if '",
            use_madgwick_filter,
            "' in ['True', 'true'] else ",
            "[-1.0, 0.0, 0.0, 0.0, 1.8e-3, 0.0, 0.0, 0.0, 1.8e-3]",  # the first element of the orientation covariance is set to -1 according to the documentation: https://docs.ros.org/en/jazzy/p/sensor_msgs/msg/Imu.html
        ]
    )

    ns_controller_config_path = ReplaceString(
        controller_config_path,
        {
            "<namespace>/": ns,
            "<static_covariance_orientation>": orientation_covariance,
        },
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
            ("drive_controller/transition_event", "drive_controller/_transition_event"),
            ("imu_broadcaster/imu", "imu/data"),
            ("imu_broadcaster/transition_event", "imu_broadcaster/_transition_event"),
            (
                "joint_state_broadcaster/transition_event",
                "joint_state_broadcaster/_transition_event",
            ),
            (
                "twist_mux_controller/transition_event",
                "twist_mux_controller/_transition_event",
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

    controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "drive_controller",
            "imu_broadcaster",
            "twist_mux_controller",
            "--activate-as-group",
            *spawner_common_args,
        ],
        namespace=namespace,
    )

    actions = [
        declare_common_dir_path_arg,
        declare_robot_model_arg,  # robot_model is used by wheel_type
        declare_use_madgwick_filter_arg,
        declare_configure_thread_priority_arg,
        declare_canopen_thread_priority_arg,
        declare_rpdo_dispatch_priority_arg,
        declare_gpio_watchdog_priority_arg,
        declare_gpio_monitor_priority_arg,
        declare_canopen_thread_cpu_arg,
        declare_rpdo_dispatch_cpu_arg,
        declare_gpio_watchdog_cpu_arg,
        declare_gpio_monitor_cpu_arg,
        declare_pdo_motor_states_timeout_ms_arg,
        declare_pdo_driver_state_timeout_ms_arg,
        declare_limit_tpdo_rates_arg,
        declare_canopen_resync_watchdog_arg,
        declare_canopen_resync_watchdog_timeout_s_arg,
        declare_wheel_type_arg,  # wheel_type is used by controller_config_path
        declare_controller_config_path_arg,
        declare_namespace_arg,
        declare_use_sim_arg,
        declare_log_level_arg,
        SetParameter(name="use_sim_time", value=use_sim),
        load_urdf,
        control_node,
        controllers_spawner,
    ]

    return LaunchDescription(actions)
