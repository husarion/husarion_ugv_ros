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

#include "husarion_ugv_manager/plugins/decorator/tick_after_timeout_node.hpp"

namespace husarion_ugv_manager
{

TickAfterTimeout::TickAfterTimeout(const std::string & name, const BT::NodeConfig & conf)
: BT::DecoratorNode(name, conf)
{
  this->last_success_time_ = std::chrono::steady_clock::now();
}

BT::NodeStatus TickAfterTimeout::tick()
{
  float timeout;
  if (!this->getInput<float>("timeout", timeout)) {
    throw(BT::RuntimeError("[", this->name(), "] Failed to get input [timeout]"));
  }

  timeout_ = std::chrono::duration<float>(timeout);

  auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    std::chrono::steady_clock::now() - last_success_time_);

  if (dt < timeout_) {
    return BT::NodeStatus::SKIPPED;
  }

  this->setStatus(BT::NodeStatus::RUNNING);
  auto child_status = this->child()->executeTick();

  if (child_status == BT::NodeStatus::SUCCESS) {
    last_success_time_ = std::chrono::steady_clock::now();
  }

  if (child_status != BT::NodeStatus::RUNNING) {
    this->resetChild();
  }

  return child_status;
}

}  // namespace husarion_ugv_manager

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<husarion_ugv_manager::TickAfterTimeout>("TickAfterTimeout");
}
