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

#ifndef HUSARION_UGV_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_
#define HUSARION_UGV_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_

#include <any>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"

#include "husarion_ugv_utils/networking_utils.hpp"

namespace husarion_ugv_manager
{

/**
 * @brief Class responsible for managing behavior trees.
 */
class BehaviorTreeManager
{
public:
  /**
   * @brief Constructor for the BehaviorTreeManager class.
   *
   * @param tree_name The name of the tree in the BehaviorTree project.
   * @param initial_blackboard A list with initial blackboard values for the tree.
   * @param groot_port The port used for the Groot2 publisher.
   */
  BehaviorTreeManager(
    const std::string & tree_name, const std::map<std::string, std::any> & initial_blackboard,
    const unsigned groot_port = 1667)
  : tree_name_(tree_name),
    initial_blackboard_(initial_blackboard),
    groot_port_(groot_port),
    tree_status_(BT::NodeStatus::IDLE)
  {
  }

  ~BehaviorTreeManager() {}

  /**
   * @brief Creates a BehaviorTree configuration, initializes the tree, and starts the Groot2
   * publisher.
   *
   * @param factory The factory object used to create the tree.
   */
  inline void Initialize(BT::BehaviorTreeFactory & factory)
  {
    config_ = CreateBTConfig(initial_blackboard_);
    tree_ = factory.createTree(tree_name_, config_.blackboard);

    const auto max_port = 65535;
    while (!husarion_ugv_utils::IsPortAvailable(groot_port_)) {
      if (groot_port_ >= max_port) {
        throw std::runtime_error("No available port for Groot2 publisher.");
      }

      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("BehaviorTreeManager"),
        "Port " << groot_port_ << " is not available. Trying next port.");

      groot_port_++;
    }

    RCLCPP_INFO_STREAM(
      rclcpp::get_logger("BehaviorTreeManager"),
      "Groot2 publisher started on port " << groot_port_);

    groot_publisher_ = std::make_unique<BT::Groot2Publisher>(tree_, groot_port_);
  }

  void TickOnce() { tree_status_ = tree_.tickOnce(); }
  void TickExactlyOnce() { tree_status_ = tree_.tickExactlyOnce(); }
  void TickWhileRunning() { tree_status_ = tree_.tickWhileRunning(); }
  void HaltTree() { tree_.haltTree(); }

  BT::NodeStatus GetTreeStatus() const { return tree_status_; }
  BT::Tree & GetTree() { return tree_; }
  BT::Blackboard::Ptr GetBlackboard() const { return config_.blackboard; }

protected:
  /**
   * @brief Creates a BehaviorTree configuration using a set of predefined blackboard values.
   *
   * @param bb_values A map containing the names of the blackboard entries and their initial values.
   * This map can include different entry types. Supported types are: bool, int, unsigned, float,
   * double, const char*, and string.
   * @exception std::invalid_argument thrown when the bb_values map contains an invalid blackboard
   * entry type.
   * @return A BehaviorTree configuration object.
   */
  inline BT::NodeConfig CreateBTConfig(const std::map<std::string, std::any> & bb_values) const
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

private:
  const std::string tree_name_;
  const std::map<std::string, std::any> initial_blackboard_;
  unsigned groot_port_;

  BT::Tree tree_;
  BT::NodeStatus tree_status_;
  BT::NodeConfig config_;
  std::unique_ptr<BT::Groot2Publisher> groot_publisher_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_BEHAVIOR_TREE_MANAGER_HPP_
