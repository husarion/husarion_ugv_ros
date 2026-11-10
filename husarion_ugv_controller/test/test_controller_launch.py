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

# Runs controller.launch.py without starting any process: every node's command line is recorded
# instead of executed. Needs the hardware packages installed (source the workspace).

import os
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
from launch_ros.substitutions import FindPackageShare

SHUTDOWN_AFTER_S = 3.0


def run_controller_launch(launch_arguments=None):
    commands = []

    def record(self, context):
        commands.append([perform_substitutions(context, part) for part in self.cmd])
        return None

    with pytest.MonkeyPatch.context() as mp:
        for name in list(os.environ):
            if name.startswith("ROBOT_"):
                mp.delenv(name)
        mp.setenv("ROBOT_MODEL_NAME", "panther")
        mp.setattr(ExecuteProcess, "execute", record)
        controller = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    FindPackageShare("husarion_ugv_controller").find("husarion_ugv_controller"),
                    "launch",
                    "controller.launch.py",
                )
            ),
            launch_arguments=(launch_arguments or {}).items(),
        )
        ls = LaunchService()
        ls.include_launch_description(
            LaunchDescription(
                [controller, TimerAction(period=SHUTDOWN_AFTER_S, actions=[Shutdown()])]
            )
        )
        return_code = ls.run()
    return return_code, commands


def control_node_params_files(commands):
    for cmd in commands:
        if cmd[0].endswith("/ros2_control_node"):
            return [cmd[i + 1] for i, arg in enumerate(cmd[:-1]) if arg == "--params-file"]
    raise AssertionError("ros2_control_node was not launched")


def read(path):
    with open(path) as file:
        return file.read()


def test_default_loads_only_the_controller_config():
    return_code, commands = run_controller_launch()
    assert return_code == 0
    params_files = control_node_params_files(commands)
    assert len(params_files) == 1, params_files
    assert "controller_manager:" in read(params_files[0])


def test_overlay_is_loaded_after_the_controller_config(tmp_path):
    overlay = tmp_path / "overlay.yaml"
    overlay.write_text(
        "/**:\n"
        "  controller_manager:\n"
        "    ros__parameters:\n"
        "      thread_priority: 60\n"
        "  imu_broadcaster:\n"
        "    ros__parameters:\n"
        "      frame_id: <namespace>/imu_link\n"
    )
    return_code, commands = run_controller_launch(
        {"controller_params_overlay_path": str(overlay), "namespace": "robot"}
    )
    assert return_code == 0
    params_files = control_node_params_files(commands)
    assert len(params_files) == 2, params_files
    assert "drive_controller:" in read(params_files[0])
    loaded_overlay = read(params_files[1])
    assert "thread_priority: 60" in loaded_overlay
    assert "frame_id: robot/imu_link" in loaded_overlay


def test_missing_overlay_fails_the_launch(tmp_path):
    missing = tmp_path / "missing.yaml"
    return_code, commands = run_controller_launch({"controller_params_overlay_path": str(missing)})
    assert return_code != 0
    assert not any(cmd[0].endswith("/ros2_control_node") for cmd in commands)


if __name__ == "__main__":
    sys.exit(pytest.main([__file__, "-v"]))
