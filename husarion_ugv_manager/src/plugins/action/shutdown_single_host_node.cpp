// Copyright 2024 Husarion sp. z o.o.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "husarion_ugv_manager/plugins/action/shutdown_single_host_node.hpp"

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/exceptions.h"
#include "behaviortree_cpp/tree_node.h"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"
#include "husarion_ugv_manager/plugins/shutdown_host.hpp"

namespace husarion_ugv_manager
{

bool ShutdownSingleHost::UpdateHosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts)
{
  std::string ip;
  if (!getInput<std::string>("ip", ip) || ip == "") {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [ip]");
    return false;
  }

  std::string user;
  if (!getInput<std::string>("username", user) || user == "") {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [user]");
    return false;
  }

  unsigned port;
  if (!getInput<unsigned>("port", port)) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [port]");
    return false;
  }

  std::string command;
  if (!getInput<std::string>("command", command) || command == "") {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [command]");
    return false;
  }

  float timeout;
  if (!getInput<float>("timeout", timeout)) {
    RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Failed to get input [timeout]");
    return false;
  }

  bool ping_for_success;
  if (!getInput<bool>("ping_for_success", ping_for_success)) {
    RCLCPP_ERROR_STREAM(
      *this->logger_, GetLoggerPrefix(name()) << "Failed to get input [ping_for_success]");
    return false;
  }

  hosts.push_back(
    std::make_shared<ShutdownHost>(ip, user, port, command, timeout, ping_for_success));
  return true;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::ShutdownSingleHost>("ShutdownSingleHost");
}
