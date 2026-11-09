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

# Starts the joy node on a FIFO that stands in for a gamepad, so it reaches the loop that reads an
# open device, then sends SIGTERM and expects a clean exit. The stock joy_linux_node ends on
# terminate here: rclcpp's handler shuts the context down, the loop exits and spin_some() then
# throws on the invalid context. The original failure is a race, and this test exercises the new
# exit path only; it cannot prove that the race is gone. Needs no hardware.

import os
import shutil
import signal
import tempfile
import unittest

import launch_testing
import launch_testing.asserts
import launch_testing.util
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_test_description():
    # Not a character device, so the sysfs check leaves it to open(). Held open read-write here, so
    # the node's read-only open() does not block and its select() just times out.
    fifo_dir = tempfile.mkdtemp()
    fifo_path = os.path.join(fifo_dir, "gamepad")
    os.mkfifo(fifo_path)
    fifo_fd = os.open(fifo_path, os.O_RDWR | os.O_NONBLOCK)

    joy_node = Node(
        package="husarion_ugv_teleop",
        executable="resilient_joy_linux_node",
        parameters=[{"dev": fifo_path, "dev_ff": ""}],
        output="screen",
    )

    return (
        # The test stops joy_node itself; KeepAliveProc keeps the launch running until it is done.
        LaunchDescription(
            [joy_node, launch_testing.util.KeepAliveProc(), launch_testing.actions.ReadyToTest()]
        ),
        {"joy_node": joy_node, "fifo_dir": fifo_dir, "fifo_fd": fifo_fd},
    )


class TestShutdown(unittest.TestCase):

    def test_sigterm_in_open_loop_exits_cleanly(self, proc_info, proc_output, joy_node):
        proc_output.assertWaitFor("Opened joystick", process=joy_node, timeout=15.0)

        os.kill(joy_node.process_details["pid"], signal.SIGTERM)
        # Up to 1 s in select(), then node teardown.
        proc_info.assertWaitForShutdown(process=joy_node, timeout=6.0)


@launch_testing.post_shutdown_test()
class TestExitCode(unittest.TestCase):

    def test_exit_code_is_zero(self, proc_info, joy_node, fifo_dir, fifo_fd):
        os.close(fifo_fd)
        shutil.rmtree(fifo_dir, ignore_errors=True)
        launch_testing.asserts.assertExitCodes(proc_info, [0], process=joy_node)
