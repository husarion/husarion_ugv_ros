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

#ifndef HUSARION_UGV_UTILS_TEST_UTILS_HPP_
#define HUSARION_UGV_UTILS_TEST_UTILS_HPP_

#include <chrono>
#include <limits>
#include <memory>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"

namespace husarion_ugv_utils::test_utils
{

/**
 * @brief Wait for ROS message to arrive
 *
 * @param node ROS node that will be spun
 * @param msg Reference to the message, it should point to a variable,
 * that will be overwritten by subscriber callback
 * @param timeout timeout to wait for message to arrive
 *
 * @return True if message was received, false if timeout was reached
 */
template <typename NodeT, typename MsgT>
bool WaitForMsg(
  const std::shared_ptr<NodeT> & node, std::shared_ptr<MsgT> & msg,
  const std::chrono::milliseconds & timeout)
{
  msg = nullptr;
  rclcpp::Time start_time = node->now();

  while (rclcpp::ok() && node->now() - start_time <= rclcpp::Duration(timeout)) {
    if (msg) {
      return true;
    }
    rclcpp::spin_some(node->get_node_base_interface());
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  return false;
}

/**
 * @brief Wait for ROS service future to arrive
 *
 * @param node ROS node that will be spun
 * @param future Reference to the service future, it should point to a variable,
 * that will be overwritten after service response
 * @param timeout timeout to wait for service response
 *
 * @return True if message was received, false if timeout was reached
 */
template <typename NodeT, typename FutureT>
bool WaitForFuture(const NodeT & node, FutureT & future, const std::chrono::milliseconds & timeout)
{
  rclcpp::Time start_time = node->now();

  while (rclcpp::ok() && node->now() - start_time <= rclcpp::Duration(timeout)) {
    rclcpp::spin_some(node->get_node_base_interface());
    if (future.wait_for(std::chrono::milliseconds(10)) == std::future_status::ready) {
      return true;
    }
  }
  return false;
}

/**
 * @brief Creates publisher for given message type, publishes the message, and spins the ROS node
 *
 * @param node ROS node that will be spun
 * @param topic_name The name of the topic for which publisher will be created
 * @param msg ROS message that will be published
 * @param qos_profile The QoS profile of the publisher
 *
 * @return True if message was received, false if timeout was reached
 */
template <typename MsgT>
void PublishAndSpin(
  const rclcpp::Node::SharedPtr & node, const std::string & topic_name, MsgT & msg,
  const rclcpp::QoS qos_profile = rclcpp::SystemDefaultsQoS())
{
  auto publisher = node->create_publisher<MsgT>(topic_name, qos_profile);

  publisher->publish(msg);

  rclcpp::spin_some(node->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

}  // namespace husarion_ugv_utils::test_utils

#endif  // HUSARION_UGV_UTILS_TEST_UTILS_HPP_
