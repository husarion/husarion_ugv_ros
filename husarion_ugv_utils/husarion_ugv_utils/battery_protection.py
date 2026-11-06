#!/usr/bin/env python3

# Copyright 2026 Husarion sp. z o.o.
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

"""Launch arguments of the battery undervoltage protection (UVP).

Every setting is a flat launch argument, so it can be passed to bringup.launch.py as
<name>:=<value>. Not passing one means its default, and uvp_enable defaults to off. The battery
driver owns the settings: it gets each one as text and checks it, so a bad value is reported in
its diagnostics and replaced by the default instead of stopping the launch.
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

# (launch argument, battery driver parameter, default, description)
UVP_ARGUMENTS = [
    (
        "uvp_enable",
        "uvp.enable",
        "False",
        "Enable the battery undervoltage protection (UVP). Off, nothing of it runs.",
    ),
    (
        "uvp_threshold_v",
        "uvp.threshold_v",
        "",
        "UVP threshold override in V, 28.0-36.0. Empty takes the HAT EEPROM value or 31.0 V.",
    ),
    (
        "uvp_charge_asap_offset_v",
        "uvp.charge_asap_offset_v",
        "0.5",
        "Charge ASAP threshold above the UVP threshold in V, 0.2-3.0.",
    ),
    (
        "uvp_consecutive_samples",
        "uvp.consecutive_samples",
        "2",
        "Consecutive raw voltage reads below a threshold that trip it, 1-10.",
    ),
    (
        "uvp_clear_hysteresis_v",
        "uvp.clear_hysteresis_v",
        "0.5",
        "Voltage above the UVP threshold, in V, that clears UVP while charging, 0.2-2.0.",
    ),
    (
        "uvp_mode",
        "uvp.mode",
        "STOP",
        "Reaction to UVP: STOP (e-stop, shutdown after uvp_stop_timeout_min without charging)"
        " or SHUTDOWN (shutdown at once).",
    ),
    (
        "uvp_stop_timeout_min",
        "uvp.stop_timeout_min",
        "5",
        "Minutes without charging before the UVP shutdown in STOP mode, 1-60. 998 turns every"
        " UVP reaction off.",
    ),
    (
        "uvp_aux_off_at_trip",
        "uvp.aux_off_at_trip",
        "False",
        "Turn the AUX power off at the UVP trip in STOP mode.",
    ),
    (
        "uvp_drive_window_s",
        "uvp.drive_window_s",
        "60",
        "Seconds the robot may drive after an e-stop reset while UVP is latched, 10-300.",
    ),
    (
        "uvp_shutdown_hosts_timeout_s",
        "uvp.shutdown_hosts_timeout_s",
        "10",
        "Upper bound in s of the wait for each user computer in the UVP shutdown, 1-30.",
    ),
    (
        "uvp_charging_debounce_s",
        "uvp.charging_debounce_s",
        "5",
        "Seconds the charging must last before it counts, 1-30.",
    ),
    (
        "uvp_hat_eeprom_path",
        "uvp.hat_eeprom_path",
        "/proc/device-tree/hat/custom_0",
        "Raw HAT EEPROM YAML with the robot.battery.uvp_threshold_v key.",
    ),
    (
        "uvp_latch_path",
        "uvp.latch_path",
        "",
        "File keeping the UVP latch across driver restarts, on a RAM file system. Empty keeps"
        " it in memory only.",
    ),
]


def declare_uvp_arguments():
    return [
        DeclareLaunchArgument(
            name,
            default_value=default,
            description=description,
            choices=["True", "true", "False", "false"] if name == "uvp_enable" else None,
        )
        for name, _, default, description in UVP_ARGUMENTS
    ]


def uvp_launch_arguments():
    """The UVP arguments, to pass on to an included launch file."""
    return {name: LaunchConfiguration(name) for name, _, _, _ in UVP_ARGUMENTS}


def uvp_parameters():
    """The battery driver parameters, each as text for the driver to check."""
    return {
        parameter: ParameterValue(LaunchConfiguration(name), value_type=str)
        for name, parameter, _, _ in UVP_ARGUMENTS
    }
