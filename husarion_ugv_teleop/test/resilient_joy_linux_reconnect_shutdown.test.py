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

# Starts the joy node on a device path that does not exist, as on a robot with a live /dev and no
# gamepad, so it stays in the reconnect loop, then sends SIGTERM and expects a clean exit. Also
# checks that the missing gamepad is reported once, not on every retry. The sysfs check for a
# placeholder node is covered by test_gamepad_utils. Needs no hardware.

import os
import shutil
import signal
import tempfile
import time
import unittest

import launch_testing
import launch_testing.asserts
import launch_testing.util
from launch import LaunchDescription
from launch_ros.actions import Node

OPEN_FAILED = "Couldn't open joystick"


def generate_test_description():
    missing_dir = tempfile.mkdtemp()

    joy_node = Node(
        package="husarion_ugv_teleop",
        executable="resilient_joy_linux_node",
        parameters=[{"dev": os.path.join(missing_dir, "js0"), "dev_ff": ""}],
        output="screen",
    )

    return (
        # The test stops joy_node itself; KeepAliveProc keeps the launch running until it is done.
        LaunchDescription(
            [joy_node, launch_testing.util.KeepAliveProc(), launch_testing.actions.ReadyToTest()]
        ),
        {"joy_node": joy_node, "missing_dir": missing_dir},
    )


class TestShutdown(unittest.TestCase):

    def test_sigterm_in_reconnect_loop_exits_cleanly(self, proc_info, proc_output, joy_node):
        proc_output.assertWaitFor(OPEN_FAILED, process=joy_node, timeout=15.0)
        # Let a few one-second retries pass, so a message logged on every retry would show.
        time.sleep(3.0)

        os.kill(joy_node.process_details["pid"], signal.SIGTERM)
        # Up to 1 s in spin_until_future_complete(), then node teardown.
        proc_info.assertWaitForShutdown(process=joy_node, timeout=6.0)


@launch_testing.post_shutdown_test()
class TestExitCode(unittest.TestCase):

    def test_exit_code_is_zero(self, proc_info, joy_node, missing_dir):
        shutil.rmtree(missing_dir, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info, [0], process=joy_node)

    def test_missing_gamepad_logged_once(self, proc_output, joy_node):
        text = "".join(output.text.decode() for output in proc_output[joy_node])
        self.assertEqual(1, text.count(OPEN_FAILED))

    def test_missing_gamepad_is_not_an_error(self, proc_output, joy_node):
        text = "".join(output.text.decode() for output in proc_output[joy_node])
        lines = [line for line in text.splitlines() if OPEN_FAILED in line]
        self.assertEqual(1, len(lines))
        self.assertTrue("[INFO]" in lines[0], lines[0])
        self.assertNotIn("[ERROR]", text)
