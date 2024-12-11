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

#include "husarion_ugv_manager/plugins/action/signal_shutdown_node.hpp"

#include <string>
#include <utility>

#include "behaviortree_cpp/basic_types.h"

namespace husarion_ugv_manager
{

BT::NodeStatus SignalShutdown::tick()
{
  const auto reason = this->getInput<std::string>("reason").value();

  std::pair<bool, std::string> signal_shutdown;
  signal_shutdown.first = true;
  signal_shutdown.second = reason;
  this->config().blackboard->set<std::pair<bool, std::string>>("signal_shutdown", signal_shutdown);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::SignalShutdown>("SignalShutdown");
}
