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

#include <husarion_ugv_manager/behavior_tree_manager.hpp>

#include <any>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "rclcpp/rclcpp.hpp"

namespace husarion_ugv_manager
{

void BehaviorTreeManager::Initialize(BT::BehaviorTreeFactory & factory)
{
  config_ = CreateBTConfig(initial_blackboard_);
  tree_ = factory.createTree(tree_name_, config_.blackboard);
  groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port_);
}

BT::NodeConfig BehaviorTreeManager::CreateBTConfig(
  const std::map<std::string, std::any> & bb_values) const
{
  BT::NodeConfig config;
  config.blackboard = BT::Blackboard::create();

  for (auto & [name, value] : bb_values) {
    const std::type_info & type = value.type();
    if (type == typeid(bool)) {
      config.blackboard->set<bool>(name, std::any_cast<bool>(value));
    } else if (type == typeid(int)) {
      config.blackboard->set<int>(name, std::any_cast<int>(value));
    } else if (type == typeid(unsigned)) {
      config.blackboard->set<unsigned>(name, std::any_cast<unsigned>(value));
    } else if (type == typeid(float)) {
      config.blackboard->set<float>(name, std::any_cast<float>(value));
    } else if (type == typeid(double)) {
      config.blackboard->set<double>(name, std::any_cast<double>(value));
    } else if (type == typeid(const char *)) {
      config.blackboard->set<std::string>(name, std::any_cast<const char *>(value));
    } else if (type == typeid(std::string)) {
      config.blackboard->set<std::string>(name, std::any_cast<std::string>(value));
    } else {
      throw std::invalid_argument(
        "Invalid type for blackboard entry. Valid types are: bool, int, unsigned, float, double, "
        "const char*, std::string");
    }
  }

  return config;
}

}  // namespace husarion_ugv_manager
