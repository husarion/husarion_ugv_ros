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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_DECORATOR_TICK_AFTER_TIMEOUT_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_DECORATOR_TICK_AFTER_TIMEOUT_NODE_HPP_

#include <chrono>
#include <string>

#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/decorator_node.h"
#include "behaviortree_cpp/tree_node.h"

namespace husarion_ugv_manager
{
class TickAfterTimeout : public BT::DecoratorNode
{
public:
  TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf);

  static BT::PortsList providedPorts()
  {
    return {BT::InputPort<float>(
      "timeout", "Time in seconds to wait before ticking the child node again.")};
  }

private:
  std::chrono::duration<float> timeout_;
  std::chrono::time_point<std::chrono::steady_clock> last_success_time_;

  BT::NodeStatus tick() override;
};
}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_DECORATOR_TICK_AFTER_TIMEOUT_NODE_HPP_
