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

#include "panther_manager/docking_manager_node.hpp"

#include <any>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <behaviortree_ros2/ros_node_params.hpp>
#include <rclcpp/rclcpp.hpp>

#include <panther_manager/behavior_tree_manager.hpp>
#include <panther_manager/behavior_tree_utils.hpp>
#include <panther_utils/moving_average.hpp>

namespace panther_manager
{

DockingManagerNode::DockingManagerNode(
  const std::string & node_name, const rclcpp::NodeOptions & options)
: Node(node_name, options)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  DeclareParameters();
  const std::map<std::string, std::any> empty_bb = {};
  const int bt_server_port = this->get_parameter("bt_server_port").as_int();

  docking_tree_manager_ = std::make_unique<BehaviorTreeManager>(
    "Docking", empty_bb, bt_server_port);

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void DockingManagerNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  RegisterBehaviorTree();
  docking_tree_manager_->Initialize(factory_);

  using namespace std::placeholders;

  const auto timer_freq = this->get_parameter("timer_frequency").as_double();
  const auto timer_period = std::chrono::duration<double>(1.0 / timer_freq);

  docking_tree_timer_ = this->create_wall_timer(
    timer_period, std::bind(&DockingManagerNode::TimerCB, this));
}

void DockingManagerNode::DeclareParameters()
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("panther_manager");
  const auto default_bt_project_path = panther_manager_pkg_path +
                                       "/behavior_trees/DockingBT.btproj";
  const std::vector<std::string> default_plugin_libs = {};

  this->declare_parameter<std::string>("bt_project_path", default_bt_project_path);
  this->declare_parameter<std::vector<std::string>>("plugin_libs", default_plugin_libs);
  this->declare_parameter<std::vector<std::string>>("ros_plugin_libs", default_plugin_libs);
  this->declare_parameter<double>("ros_communication_timeout.availability", 1.0);
  this->declare_parameter<double>("ros_communication_timeout.response", 1.0);

  this->declare_parameter<float>("timer_frequency", 20.0);
  this->declare_parameter<int>("bt_server_port", 4444);
}

void DockingManagerNode::RegisterBehaviorTree()
{
  const auto bt_project_path = this->get_parameter("bt_project_path").as_string();

  const auto plugin_libs = this->get_parameter("plugin_libs").as_string_array();
  const auto ros_plugin_libs = this->get_parameter("ros_plugin_libs").as_string_array();

  const auto service_availability_timeout =
    this->get_parameter("ros_communication_timeout.availability").as_double();
  const auto service_response_timeout =
    this->get_parameter("ros_communication_timeout.response").as_double();

  BT::RosNodeParams params;
  params.nh = this->shared_from_this();
  auto wait_for_server_timeout_s = std::chrono::duration<double>(service_availability_timeout);
  params.wait_for_server_timeout =
    std::chrono::duration_cast<std::chrono::milliseconds>(wait_for_server_timeout_s);
  auto server_timeout_s = std::chrono::duration<double>(service_response_timeout);
  params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(server_timeout_s);

  behavior_tree_utils::RegisterBehaviorTree(
    factory_, bt_project_path, plugin_libs, params, ros_plugin_libs);

  RCLCPP_INFO_STREAM(
    this->get_logger(), "BehaviorTree registered from path '" << bt_project_path << "'");
}

void DockingManagerNode::TimerCB()
{
  docking_tree_manager_->TickOnce();

  if (docking_tree_manager_->GetTreeStatus() == BT::NodeStatus::FAILURE) {
    RCLCPP_WARN(this->get_logger(), "Docking behavior tree returned FAILURE status");
  }
}

}  // namespace panther_manager
