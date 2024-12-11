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

#ifndef HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
#define HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_

#include <any>
#include <map>
#include <stdexcept>
#include <string>
#include <typeinfo>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/tree_node.h"
#include "behaviortree_cpp/utils/shared_library.h"
#include "behaviortree_ros2/plugins.hpp"

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
}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_BEHAVIOR_TREE_UTILS_HPP_
