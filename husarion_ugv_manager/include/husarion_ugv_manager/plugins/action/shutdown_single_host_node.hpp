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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_SINGLE_HOST_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_SINGLE_HOST_NODE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"

#include "husarion_ugv_manager/plugins/shutdown_host.hpp"
#include "husarion_ugv_manager/plugins/shutdown_hosts_node.hpp"

namespace husarion_ugv_manager
{

class ShutdownSingleHost : public ShutdownHosts
{
public:
  ShutdownSingleHost(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>("ip", "IP address of the host to shut down."),
      BT::InputPort<std::string>(
        "username", "Username to use for logging in and executing the shutdown command."),
      BT::InputPort<unsigned>("port", "SSH port used for communication (default is usually 22)."),
      BT::InputPort<std::string>("command", "A command to execute on the remote host."),
      BT::InputPort<float>(
        "timeout", "Maximum time (in seconds) to wait for the host to shut down."),
      BT::InputPort<bool>(
        "ping_for_success",
        "Whether to continuously ping the host until it becomes unreachable or the timeout is "
        "reached.")};
  }

private:
  bool UpdateHosts(std::vector<std::shared_ptr<ShutdownHost>> & hosts) override;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_SINGLE_HOST_NODE_HPP_
