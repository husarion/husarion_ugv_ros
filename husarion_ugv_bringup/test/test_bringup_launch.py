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

# Runs the whole hardware bringup without starting any process: every node's command line is
# recorded instead of executed. Needs the hardware packages installed (source the workspace).

import os
import pathlib
import sys

import pytest
from launch import LaunchDescription, LaunchService
from launch.actions import (
    ExecuteProcess,
    IncludeLaunchDescription,
    Shutdown,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.substitutions import FindPackageShare

# bringup.launch.py starts battery, lights, manager and ekf from a 10 s TimerAction.
SHUTDOWN_AFTER_S = 12.0


def run_bringup():
    commands = []

    def record(self, context):
        cmd = [perform_substitutions(context, part) for part in self.cmd]
        commands.append(cmd)
        return None

    original_execute = ExecuteProcess.execute
    original_load = LoadComposableNodes.execute
    ExecuteProcess.execute = record
    LoadComposableNodes.execute = lambda self, context: None
    try:
        bringup = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare("husarion_ugv_bringup").find("husarion_ugv_bringup"),
                    "launch",
                    "bringup.launch.py",
                )
            )
        )
        ls = LaunchService()
        ls.include_launch_description(
            LaunchDescription(
                [bringup, TimerAction(period=SHUTDOWN_AFTER_S, actions=[Shutdown()])]
            )
        )
        ls.run()
    finally:
        ExecuteProcess.execute = original_execute
        LoadComposableNodes.execute = original_load
    return commands


def params_files(cmd):
    return [cmd[i + 1] for i, arg in enumerate(cmd[:-1]) if arg == "--params-file"]


@pytest.fixture(scope="module")
def commands():
    os.environ["ROBOT_HW_CONFIG_CORRECT"] = "true"
    os.environ.setdefault("ROBOT_MODEL_NAME", "panther")
    return run_bringup()


def find_node(commands, name):
    for cmd in commands:
        if f"__node:={name}" in cmd:
            return cmd
    raise AssertionError(f"{name} was not launched")


def test_battery_driver_gets_package_battery_yaml(commands):
    # load_urdf.launch.py declares a simulation-only battery_config_path with an empty default.
    # When that leaks out of controller.launch.py it wins over battery.launch.py's own default.
    cmd = find_node(commands, "battery_driver")
    expected = pathlib.Path(
        FindPackageShare("husarion_ugv_battery").find("husarion_ugv_battery"),
        "config",
        "battery.yaml",
    )
    assert str(expected) in params_files(cmd), cmd


def test_every_params_file_exists(commands):
    for cmd in commands:
        for path in params_files(cmd):
            assert os.path.isfile(path), cmd


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
