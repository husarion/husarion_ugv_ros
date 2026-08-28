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

"""DEPRECATED backwards-compatibility shim for `simulation.launch.yaml`.

External tutorials and Husarion manuals reference `simulation.launch.py`. The actual logic
lives in `simulation.launch.yaml`. New code should reference the YAML file directly.
"""

import warnings

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

_DEPRECATION_MSG = (
    "\033[33;1m"  # bold yellow
    "[DEPRECATED] 'simulation.launch.py' is a backwards-compatibility shim and will be "
    "removed in a future release. Use 'simulation.launch.yaml' directly:\n"
    "  ros2 launch husarion_ugv_gazebo simulation.launch.yaml"
    "\033[0m"
)

warnings.warn(
    "simulation.launch.py is deprecated; use simulation.launch.yaml directly.",
    DeprecationWarning,
    stacklevel=2,
)

gazebo_pkg = FindPackageShare("husarion_ugv_gazebo")


def generate_launch_description():
    return LaunchDescription(
        [
            LogInfo(msg=_DEPRECATION_MSG),
            IncludeLaunchDescription(
                PathJoinSubstitution([gazebo_pkg, "launch", "simulation.launch.yaml"])
            ),
        ]
    )
