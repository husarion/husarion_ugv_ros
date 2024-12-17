#!/usr/bin/env python3

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

import unittest

import launch
import launch_testing
import pytest
from diagnostic_msgs.msg import DiagnosticArray
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_testing_ros import WaitForTopics

from husarion_ugv_msgs.msg import SystemStatus


@pytest.mark.launch_test
def generate_test_description():

    system_monitor_node = Node(
        package="husarion_ugv_diagnostics",
        executable="system_monitor_node",
    )

    # Start test after 1s
    delay_timer = launch.actions.TimerAction(
        period=1.0, actions=[launch_testing.actions.ReadyToTest()]
    )

    actions = [system_monitor_node, delay_timer]

    context = {}

    return (
        LaunchDescription(actions),
        context,
    )


class TestNodeInitialization(unittest.TestCase):
    def test_initialization_log(self, proc_output):
        proc_output.assertWaitFor("Initialized successfully.")


class TestNode(unittest.TestCase):
    def test_msg(self):
        topic_list = [("system_status", SystemStatus), ("diagnostics", DiagnosticArray)]

        with WaitForTopics(topic_list, timeout=5.0) as wait_for_topics:
            received_topics_str = ", ".join(wait_for_topics.topics_received())
            print("Received messages from the following topics: [" + received_topics_str + "]")


@launch_testing.post_shutdown_test()
class TestProcessOutput(unittest.TestCase):
    def test_exit_code(self, proc_info):
        # Check that process exits with code 0: no error
        launch_testing.asserts.assertExitCodes(proc_info)
