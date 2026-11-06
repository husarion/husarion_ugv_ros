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


def run_bringup(launch_arguments=None):
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
            ),
            launch_arguments=(launch_arguments or {}).items(),
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


def params_text(cmd):
    text = ""
    for path in params_files(cmd):
        with open(path) as params_file:
            text += params_file.read()
    return text


def robot_description(commands):
    for cmd in commands:
        if cmd[0].endswith("/robot_state_publisher"):
            return params_text(cmd)
    raise AssertionError("robot_state_publisher was not launched")


@pytest.fixture(scope="module")
def uvp_commands():
    os.environ["ROBOT_HW_CONFIG_CORRECT"] = "true"
    os.environ.setdefault("ROBOT_MODEL_NAME", "panther")
    return run_bringup(
        {
            "uvp_enable": "true",
            "uvp_threshold_v": "32.5",
            "uvp_mode": "SHUTDOWN",
            "uvp_latch_path": "/run/husarion/uvp_latch",
        }
    )


def test_uvp_off_leaves_robot_description_unchanged(commands):
    description = robot_description(commands)
    assert "<ros2_control" in description
    assert "uvp_enable" not in description


def test_uvp_off_managers_and_battery_driver_inert(commands):
    for name in ("safety_manager", "lights_manager"):
        assert "uvp_enable: false" in params_text(find_node(commands, name))
    battery = params_text(find_node(commands, "battery_driver"))
    assert "uvp.enable: 'False'" in battery, battery


def test_uvp_on_reaches_every_part(uvp_commands):
    assert '<param name="uvp_enable">true</param>' in robot_description(uvp_commands)
    for name in ("safety_manager", "lights_manager"):
        assert "uvp_enable: true" in params_text(find_node(uvp_commands, name))
    battery = params_text(find_node(uvp_commands, "battery_driver"))
    assert "uvp.enable: 'true'" in battery, battery
    assert "uvp.threshold_v: '32.5'" in battery, battery
    assert "uvp.mode: SHUTDOWN" in battery, battery
    assert "uvp.latch_path: /run/husarion/uvp_latch" in battery, battery
    # Not passed, so the default.
    assert "uvp.stop_timeout_min: '5'" in battery, battery


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
