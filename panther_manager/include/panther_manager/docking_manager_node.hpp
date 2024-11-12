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

#ifndef PANTHER_MANAGER_DOCKING_MANAGER_NODE_HPP_
#define PANTHER_MANAGER_DOCKING_MANAGER_NODE_HPP_

#include <memory>
#include <string>

#include <behaviortree_cpp/bt_factory.h>
#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/bool.hpp>

#include "panther_manager/behavior_tree_manager.hpp"
#include "panther_utils/moving_average.hpp"

namespace panther_manager
{

using BoolMsg = std_msgs::msg::Bool;

/**
 * @brief This class is responsible for creating a BehaviorTree responsible for docking management,
 * spinning it, and updating blackboard entries based on subscribed topics.
 */
class DockingManagerNode : public rclcpp::Node
{
public:
  DockingManagerNode(
    const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~DockingManagerNode() = default;

  /**
   * @brief Initializes the docking manager, setting up parameters and behavior tree.
   * @throws std::runtime_error if initialization fails
   */
  void Initialize();

protected:
  void DeclareParameters();
  void RegisterBehaviorTree();

  std::unique_ptr<BehaviorTreeManager> docking_tree_manager_;

private:
  void TimerCB();

  rclcpp::TimerBase::SharedPtr docking_tree_timer_;

  BT::BehaviorTreeFactory factory_;
};

}  // namespace panther_manager

#endif  // PANTHER_MANAGER_DOCKING_MANAGER_NODE_HPP_
