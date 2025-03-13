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

#ifndef HUSARION_UGV_MANAGER_TEST_UTILS_BEHAVIOR_TREE_TEST_UTILS_HPP_
#define HUSARION_UGV_MANAGER_TEST_UTILS_BEHAVIOR_TREE_TEST_UTILS_HPP_

#include <chrono>
#include <functional>
#include <thread>

#include <gtest/gtest.h>

#include "behaviortree_cpp/basic_types.h"

#include "rclcpp/rclcpp.hpp"

namespace behavior_tree::test_utils
{

/**
 * @brief Spin ROS node while behavior tree status is either RUNNING, SKIPPED, or IDLE.
 *
 * @param node Ros node that will be spun.
 * @param GetStatus method that will be used to obtain current tree status.
 * @return False if tree status changed to FAILURE or timeout was reached, true otherwise.
 */
bool SpinWhileRunning(
  const rclcpp::Node::SharedPtr node, std::function<BT::NodeStatus()> GetStatus,
  const std::chrono::milliseconds & timeout)
{
  const rclcpp::Time start_time = node->now();

  auto status = BT::NodeStatus::RUNNING;
  while (rclcpp::ok() && !BT::isStatusCompleted(status) &&
         node->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(node);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    status = GetStatus();
  }

  if (status == BT::NodeStatus::SUCCESS) {
    return true;
  }
  return false;
}

}  // namespace behavior_tree::test_utils

#endif  // HUSARION_UGV_MANAGER_TEST_UTILS_BEHAVIOR_TREE_TEST_UTILS_HPP_
