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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_

#include <any>
#include <chrono>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>
#include <vector>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"
#include "rclcpp/rclcpp.hpp"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace husarion_ugv_manager::behavior_tree_utils
{

/**
 * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes loaded
 * from plugins.
 *
 * @param factory The BT factory used to register nodes and the BehaviorTree.
 * @param bt_project_path The path to the BehaviorTree project file.
 * @param plugin_libs A vector containing the names of the nodes that will be registered from
 * plugins.
 */
inline void RegisterBehaviorTree(
  BT::BehaviorTreeFactory & factory, const std::string & bt_project_path,
  const std::vector<std::string> plugin_libs)
{
  for (const auto & plugin : plugin_libs) {
    factory.registerFromPlugin(BT::SharedLibrary::getOSName(plugin));
  }

  factory.registerBehaviorTreeFromFile(bt_project_path);
}

/**
 * @brief Registers a BehaviorTree into the factory from a file, with custom BT nodes and ROS nodes
 * loaded from plugins.
 *
 * @param factory The BT factory used to register nodes and the BehaviorTree.
 * @param bt_project_path The path to the BehaviorTree project file.
 * @param plugin_libs A vector containing the names of the nodes that will be registered from
 * plugins.
 * @param params The ROS-related parameters to register nodes.
 * @param ros_plugin_libs A vector containing the names of the ROS nodes that will be registered
 * from plugins.
 */
inline void RegisterBehaviorTree(
  BT::BehaviorTreeFactory & factory, const std::string & bt_project_path,
  const std::vector<std::string> plugin_libs, const BT::RosNodeParams & params,
  const std::vector<std::string> ros_plugin_libs)
{
  for (const auto & plugin : ros_plugin_libs) {
    RegisterRosNode(factory, BT::SharedLibrary::getOSName(plugin), params);
  }

  RegisterBehaviorTree(factory, bt_project_path, plugin_libs);
}

/**
 * @brief Creates RosNodeParams from the blackboard contained in the NodeConfig.
 *
 * @param conf The NodeConfig containing the blackboard.
 * @return BT::RosNodeParams The created RosNodeParams.
 *
 * @throw std::runtime_error Throws when the blackboard is nullptr or the node
 * (rclcpp::Node::SharedPtr) is not found or is nullptr.
 */
inline BT::RosNodeParams CreateRosNodeParamsFromBlackboard(const BT::NodeConfig & conf)
{
  auto blackboard = conf.blackboard;
  if (!blackboard) {
    throw std::runtime_error("Blackboard is nullptr");
  }

  rclcpp::Node::SharedPtr node;

  try {
    node = blackboard->get<rclcpp::Node::SharedPtr>("node");
  } catch (const std::exception & e) {
    throw BT::RuntimeError("Failed to get rclcpp::Node from blackboard: ", e.what());
  }

  if (!node) {
    throw BT::RuntimeError("rclcpp::Node is nullptr");
  }

  return BT::RosNodeParams(node);
}

/**
 * @brief Waits once for services that the tree calls, spinning the node meanwhile. Each service
 * node of a tree waits only briefly for its service while the tree is built and logs an error if it
 * is missing, so a server that starts together with the manager would be reported as not reachable.
 *
 * @param node The node, which must not be added to another executor yet.
 * @param service_names The names of the services, resolved like the node's own clients (namespace
 * and remapping).
 * @param timeout Time to wait for all of them.
 * @return True if every service has a server within the timeout. A timeout is logged as a warning.
 * False also when the node's context was shut down during the wait.
 */
inline bool WaitForServices(
  const rclcpp::Node::SharedPtr & node, const std::vector<std::string> & service_names,
  const std::chrono::duration<double> & timeout)
{
  std::vector<std::string> resolved_names;
  for (const auto & name : service_names) {
    resolved_names.push_back(
      node->get_node_base_interface()->resolve_topic_or_service_name(name, true));
  }
  const auto missing_services = [&]() {
    std::string missing;
    for (const auto & name : resolved_names) {
      if (node->count_services(name) == 0) {
        missing += (missing.empty() ? "'" : ", '") + name + "'";
      }
    }
    return missing;
  };

  const auto context = node->get_node_base_interface()->get_context();
  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context;
  rclcpp::executors::SingleThreadedExecutor executor(executor_options);
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  auto missing = missing_services();
  bool waiting_logged = false;
  while (!missing.empty()) {
    if (std::chrono::steady_clock::now() >= deadline) {
      RCLCPP_WARN(
        node->get_logger(), "Services not available after %.1f s: %s", timeout.count(),
        missing.c_str());
      break;
    }
    executor.spin_once(std::chrono::milliseconds(50));
    // A callback, or a signal, may have shut the context down, and the graph cannot be read then.
    if (!context->is_valid()) {
      break;
    }
    missing = missing_services();
    if (!missing.empty() && !waiting_logged) {
      RCLCPP_INFO(node->get_logger(), "Waiting for services: %s", missing.c_str());
      waiting_logged = true;
    }
  }
  executor.remove_node(node);
  return context->is_valid() && missing.empty();
}

}  // namespace husarion_ugv_manager::behavior_tree_utils

namespace husarion_ugv_manager
{
// TODO: @pawelirh move to a separate file with an appropriate abstraction layer
/**
 * @brief Get the Logger Prefix of BT node
 *
 * @param bt_node_name Behavior tree node name
 * @return std::string Logger prefix
 */
inline std::string GetLoggerPrefix(const std::string & bt_node_name)
{
  return std::string("[" + bt_node_name + "] ");
}

inline bool TimeoutExceeded(
  const std::chrono::time_point<std::chrono::steady_clock> & start_time,
  const std::chrono::milliseconds & timeout)
{
  return std::chrono::steady_clock::now() - start_time > timeout;
}
}  // namespace husarion_ugv_manager

namespace BT
{
/**
 * @brief Converts a string to a vector of float.
 *
 * @param str The string to convert.
 * @return std::vector<float> The vector of float.
 *
 * @throw BT::RuntimeError Throws when there is no input or cannot parse float.
 */
template <>
inline std::vector<float> convertFromString<std::vector<float>>(StringView str)
{
  auto parts = splitString(str, ';');
  std::vector<float> output;
  output.reserve(parts.size());
  for (const StringView & part : parts) {
    output.push_back(convertFromString<float>(part));
  }
  return output;
}

/**
 * @brief Converts a string to a PoseStamped message.
 *
 * The string format should be "x;y;z;roll;pitch;yaw;frame_id" where:
 *  - x, y, z: Position coordinates.
 *  - roll, pitch, yaw: Rotation around axes XYZ.
 *  - frame_id: Coordinate frame ID (string).
 *
 * @param str The string to convert.
 * @return geometry_msgs::msg::PoseStamped The converted PoseStamped message.
 *
 * @throw BT::RuntimeError Throws if the input is invalid or cannot be parsed.
 */
template <>
inline geometry_msgs::msg::PoseStamped convertFromString<geometry_msgs::msg::PoseStamped>(
  StringView str)
{
  constexpr std::size_t expected_parts_size = 7;

  auto parts = splitString(str, ';');
  if (parts.size() != expected_parts_size) {
    throw BT::RuntimeError(
      "Invalid input for PoseStamped. Expected " + std::to_string(expected_parts_size) +
      " values: x;y;z;roll;pitch;yaw;frame_id");
  }

  geometry_msgs::msg::PoseStamped pose_stamped;

  try {
    pose_stamped.pose.position.x = convertFromString<double>(parts[0]);
    pose_stamped.pose.position.y = convertFromString<double>(parts[1]);
    pose_stamped.pose.position.z = convertFromString<double>(parts[2]);

    const auto roll = convertFromString<double>(parts[3]);
    const auto pitch = convertFromString<double>(parts[4]);
    const auto yaw = convertFromString<double>(parts[5]);
    tf2::Quaternion quaternion;
    quaternion.setRPY(roll, pitch, yaw);
    pose_stamped.pose.orientation = tf2::toMsg(quaternion);

    pose_stamped.header.frame_id = convertFromString<std::string>(parts[6]);
    pose_stamped.header.stamp = rclcpp::Clock().now();

  } catch (const std::exception & e) {
    throw BT::RuntimeError("Failed to convert string to PoseStamped: " + std::string(e.what()));
  }

  return pose_stamped;
}

}  // namespace BT

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
