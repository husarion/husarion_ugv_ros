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

import subprocess
from typing import List


class ROSNodeInfo:
    """
    Class representing the ROS node info.
    """

    def __init__(self):
        self.subscribers: List[str] = []
        self.publishers: List[str] = []
        self.service_servers: List[str] = []
        self.service_clients: List[str] = []
        self.action_servers: List[str] = []
        self.action_clients: List[str] = []


def get_node_info(node_name: str) -> ROSNodeInfo:
    """
    Executes the command 'ros2 node info <node_name>' and returns the ROSNodeInfo object.

    Args:
        node_name (str): The name of the ROS 2 node to get information about.

    Returns:
        ROSNodeInfo: An object representing a complete node info.

    Raises:
        RuntimeError: If the command execution fails.
    """
    node_info = ROSNodeInfo()

    section_map = {
        "Subscribers:": node_info.subscribers,
        "Publishers:": node_info.publishers,
        "Service Servers:": node_info.service_servers,
        "Service Clients:": node_info.service_clients,
        "Action Servers:": node_info.action_servers,
        "Action Clients:": node_info.action_clients,
    }

    try:
        # Execute the `ros2 node info` command
        result = subprocess.run(
            ["ros2", "node", "info", node_name], capture_output=True, text=True, check=True
        )

        # Parse the output
        lines = result.stdout.splitlines()
        current_section = None

        for line in lines:
            line = line.strip()
            if line in section_map:
                current_section = section_map[line]
            elif line and current_section is not None:
                current_section.append(line.split(":")[0].strip())
            else:
                current_section = None

    except subprocess.CalledProcessError as e:
        raise RuntimeError(f"Error executing command: {e}. stderr: {e.stderr}") from e

    return node_info
