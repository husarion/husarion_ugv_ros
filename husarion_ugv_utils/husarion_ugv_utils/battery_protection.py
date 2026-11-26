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
<name>:=<value>. An argument left empty passes nothing, so the battery YAML or the driver default
applies. uvp_enable is always passed and defaults to off, as it also switches the hardware system
and the managers. The battery driver owns the settings: it gets each one as text and checks it, so
a bad value is reported in its diagnostics and replaced by the default instead of stopping the
launch.
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
        "uvp_reactions_enabled",
        "uvp.reactions_enabled",
        "",
        "False turns every UVP reaction off: no e-stop, no shutdown, no AUX power off and no charge"
        " ASAP animation. The detection and its reporting stay."
        " Unset: the battery YAML, or True.",
    ),
    (
        "uvp_threshold_v",
        "uvp.threshold_v",
        "",
        "UVP threshold override in V, 28.0-36.0. Unset takes the HAT EEPROM value or 31.0 V.",
    ),
    (
        "uvp_charge_asap_offset_v",
        "uvp.charge_asap_offset_v",
        "",
        "Charge ASAP threshold above the UVP threshold in V, 0.2-3.0."
        " Unset: the battery YAML, or 0.5.",
    ),
    (
        "uvp_consecutive_samples",
        "uvp.consecutive_samples",
        "",
        "Consecutive raw voltage reads below a threshold that trip it, 1-10."
        " Unset: the battery YAML, or 2.",
    ),
    (
        "uvp_clear_hysteresis_v",
        "uvp.clear_hysteresis_v",
        "",
        "Voltage above the UVP threshold, in V, that clears UVP while charging, 0.2-2.0."
        " Unset: the battery YAML, or 0.5.",
    ),
    (
        "uvp_mode",
        "uvp.mode",
        "",
        "Reaction to UVP: STOP (e-stop, shutdown after uvp_stop_timeout_min without charging)"
        " or SHUTDOWN (shutdown once the robot has not charged for 10 s)."
        " Unset: the battery YAML, or STOP.",
    ),
    (
        "uvp_stop_timeout_min",
        "uvp.stop_timeout_min",
        "",
        "Minutes without charging before the UVP shutdown in STOP mode, 1-60. 998 turns the"
        " reactions off, kept for compatibility with existing configurations."
        " Unset: the battery YAML, or 5.",
    ),
    (
        "uvp_aux_off_at_trip",
        "uvp.aux_off_at_trip",
        "",
        "Turn the AUX power off at the UVP trip in STOP mode."
        " Unset: the battery YAML, or False.",
    ),
    (
        "uvp_drive_window_s",
        "uvp.drive_window_s",
        "",
        "Seconds the robot may drive after an e-stop reset while UVP is latched, 10-300."
        " Unset: the battery YAML, or 60.",
    ),
    (
        "uvp_shutdown_hosts_timeout_s",
        "uvp.shutdown_hosts_timeout_s",
        "",
        "Upper bound in s of the wait for each user computer in the UVP shutdown, 1-30."
        " Unset: the battery YAML, or 10.",
    ),
    (
        "uvp_charging_debounce_s",
        "uvp.charging_debounce_s",
        "",
        "Seconds the charging must last before it counts, 1-30. Unset: the battery YAML, or 5.",
    ),
    (
        "uvp_hat_eeprom_path",
        "uvp.hat_eeprom_path",
        "",
        "Raw HAT EEPROM YAML with the robot.battery.uvp_threshold_v key."
        " Unset: the battery YAML, or /proc/device-tree/hat/custom_0.",
    ),
    (
        "uvp_latch_path",
        "uvp.latch_path",
        "",
        "File keeping the UVP latch across driver restarts, on a RAM file system. Unset keeps"
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


def uvp_parameters(context):
    """The battery driver parameters that are set, each as text for the driver to check.

    Evaluated in an OpaqueFunction: an empty argument is left out, so it does not override the
    battery YAML.
    """
    parameters = {}
    for name, parameter, _, _ in UVP_ARGUMENTS:
        value = LaunchConfiguration(name).perform(context)
        if value != "" or name == "uvp_enable":
            parameters[parameter] = ParameterValue(value, value_type=str)
    return parameters
