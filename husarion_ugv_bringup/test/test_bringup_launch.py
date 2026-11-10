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
from launch.substitutions import TextSubstitution
from launch.utilities import perform_substitutions
from launch_ros.actions import LoadComposableNodes
from launch_ros.substitutions import FindPackageShare

from husarion_ugv_utils import messages

# bringup.launch.py starts battery, lights, manager and ekf from a 10 s TimerAction. The margin
# covers a slow CI runner; on a timeout find_node says which node is missing.
SHUTDOWN_AFTER_S = 15.0


def run_bringup(launch_arguments=None):
    # Records each node's command line instead of starting it. Relies on Node.execute reaching
    # ExecuteProcess.execute, and on self.cmd still holding the unperformed substitutions.
    commands = []

    def record(self, context):
        cmd = [perform_substitutions(context, part) for part in self.cmd]
        commands.append(cmd)
        return None

    with pytest.MonkeyPatch.context() as mp:
        # The welcome message reads the husarion_ugv version with a Command, and that metapackage
        # is not on the path while this package's own tests run.
        mp.setattr(messages, "Command", lambda command: TextSubstitution(text="0.0.0"))
        # Run at the launch files' defaults, not at the configuration of the robot running it.
        for name in list(os.environ):
            if name.startswith("ROBOT_") or name == "SYSTEM_BUILD_VERSION":
                mp.delenv(name)
        mp.setenv("ROBOT_HW_CONFIG_CORRECT", "true")
        mp.setenv("ROBOT_MODEL_NAME", "panther")
        mp.setattr(ExecuteProcess, "execute", record)
        mp.setattr(LoadComposableNodes, "execute", lambda self, context: None)
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
    return commands


def params_files(cmd):
    return option_values(cmd, "--params-file")


@pytest.fixture(scope="module")
def commands():
    return run_bringup()


def find_node(commands, name):
    # ros2_control_node and robot_state_publisher have no name remap, so match them by executable.
    for cmd in commands:
        if f"__node:={name}" in cmd or cmd[0].endswith(f"/{name}"):
            return cmd
    raise AssertionError(
        f"{name} was not launched (nodes started by the bringup timer need it to fire within"
        f" {SHUTDOWN_AFTER_S} s)"
    )


def option_values(cmd, option):
    return [cmd[i + 1] for i, arg in enumerate(cmd[:-1]) if arg == option]


def test_key_nodes_are_recorded(commands):
    # Guards the recorder itself: if a launch_ros change stops ExecuteProcess.execute from being
    # reached, every other test here would pass on an empty list.
    for name in [
        "robot_state_publisher",
        "ros2_control_node",
        "battery_driver",
        "lights_container",
        "safety_manager",
    ]:
        find_node(commands, name)


def test_log_level_warning_reaches_nodes_as_warn():
    commands = run_bringup({"log_level": "WARNING"})
    find_node(commands, "safety_manager")
    for cmd in commands:
        for level in option_values(cmd, "--log-level"):
            assert level != "WARNING" and not level.endswith(":=WARNING"), cmd
    cmd = find_node(commands, "ros2_control_node")
    levels = option_values(cmd, "--log-level")
    assert "WARN" in levels, cmd
    assert "rcl:=WARN" in levels, cmd


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
    # Plain nodes only: LoadComposableNodes is patched out, so the lights yamls are not checked.
    assert commands, "bringup launched nothing"
    for cmd in commands:
        for path in params_files(cmd):
            assert os.path.isfile(path), cmd


def test_controller_params_overlay_reaches_ros2_control_node(tmp_path):
    overlay = tmp_path / "overlay.yaml"
    overlay.write_text(
        "/**:\n  controller_manager:\n    ros__parameters:\n      lock_memory: true\n"
    )
    commands = run_bringup({"controller_params_overlay_path": str(overlay)})
    files = params_files(find_node(commands, "ros2_control_node"))
    assert len(files) == 2, files
    with open(files[1]) as file:
        assert "lock_memory: true" in file.read()


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
