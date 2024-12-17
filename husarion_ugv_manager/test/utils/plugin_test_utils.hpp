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

#ifndef HUSARION_UGV_MANAGER_PLUGIN_TEST_UTILS_HPP_
#define HUSARION_UGV_MANAGER_PLUGIN_TEST_UTILS_HPP_

#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "husarion_ugv_msgs/srv/set_led_animation.hpp"

#include "husarion_ugv_manager/plugins/action/call_set_bool_service_node.hpp"
#include "husarion_ugv_manager/plugins/action/call_set_led_animation_service_node.hpp"
#include "husarion_ugv_manager/plugins/action/call_trigger_service_node.hpp"
#include "husarion_ugv_manager/plugins/action/shutdown_hosts_from_file_node.hpp"
#include "husarion_ugv_manager/plugins/action/shutdown_single_host_node.hpp"
#include "husarion_ugv_manager/plugins/action/signal_shutdown_node.hpp"
#include "husarion_ugv_manager/plugins/decorator/tick_after_timeout_node.hpp"

namespace husarion_ugv_manager::plugin_test_utils
{

struct BehaviorTreePluginDescription
{
  std::string name;
  std::map<std::string, std::string> params;
};

class PluginTestUtils : public testing::Test
{
public:
  PluginTestUtils()
  {
    rclcpp::init(0, nullptr);
    bt_node_ = std::make_shared<rclcpp::Node>("test_husarion_ugv_manager_node");
  }

  ~PluginTestUtils()
  {
    bt_node_.reset();
    rclcpp::shutdown();
    if (executor_thread_) {
      executor_.reset();
      executor_thread_->join();
    }
  }

  virtual std::string BuildBehaviorTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & bb_ports)
  {
    std::stringstream bt;

    bt << tree_header_ << std::endl;

    bt << "\t\t\t\t<" << plugin_name << " ";

    for (auto const & [key, value] : bb_ports) {
      bt << key << "=\"" << value << "\" ";
    }

    bt << " />" << std::endl;

    bt << tree_footer_;

    return bt.str();
  }

  void CreateTree(
    const std::string & plugin_name, const std::map<std::string, std::string> & bb_ports)
  {
    auto xml_text = BuildBehaviorTree(plugin_name, bb_ports);
    tree_ = factory_.createTreeFromText(xml_text);
  }

  inline BT::Tree & GetTree() { return tree_; }

  inline BT::BehaviorTreeFactory & GetFactory() { return factory_; }

  template <typename ServiceT>
  void CreateService(
    const std::string & service_name,
    std::function<
      void(const typename ServiceT::Request::SharedPtr, typename ServiceT::Response::SharedPtr)>
      service_callback)
  {
    server_node_ = std::make_shared<rclcpp::Node>("test_node_for_" + service_name);
    service_server_ = server_node_->create_service<ServiceT>(service_name, service_callback);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_);
    executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
  }

  template <typename ActionT>
  void CreateAction(
    const std::string & action_name,
    std::function<rclcpp_action::GoalResponse(
      const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const typename ActionT::Goal> goal)>
      handle_goal,
    std::function<rclcpp_action::CancelResponse(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)>
      handle_cancel,
    std::function<void(const std::shared_ptr<rclcpp_action::ServerGoalHandle<ActionT>> goal_handle)>
      handle_accepted)
  {
    server_node_ = std::make_shared<rclcpp::Node>("test_node_for_" + action_name);
    action_server_ = rclcpp_action::create_server<ActionT>(
      server_node_, action_name, handle_goal, handle_cancel, handle_accepted);
    executor_ = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(server_node_);
    executor_thread_ = std::make_unique<std::thread>([this]() { executor_->spin(); });
  }

  template <typename BTNodeT>
  void RegisterNodeWithParams(const std::string & node_type_name)
  {
    BT::RosNodeParams params;
    params.nh = bt_node_;

    factory_.registerNodeType<BTNodeT>(node_type_name, params);
  }

  template <typename BTNodeT>
  void RegisterNodeWithoutParams(const std::string & node_type_name)
  {
    factory_.registerNodeType<BTNodeT>(node_type_name);
  }

protected:
  rclcpp::Node::SharedPtr bt_node_;
  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;

  rclcpp::Node::SharedPtr server_node_;
  rclcpp::executors::SingleThreadedExecutor::UniquePtr executor_;

  rclcpp::ServiceBase::SharedPtr service_server_;
  rclcpp_action::ServerBase::SharedPtr action_server_;
  std::unique_ptr<std::thread> executor_thread_;

  inline void SpinExecutor() { executor_->spin(); }

  const std::string tree_header_ = R"(
      <root BTCPP_format="4">
        <BehaviorTree>
          <Sequence>
  )";

  const std::string tree_footer_ = R"(
            </Sequence>
        </BehaviorTree>
      </root>
  )";
};
}  // namespace husarion_ugv_manager::plugin_test_utils
#endif  // HUSARION_UGV_MANAGER_PLUGIN_TEST_UTILS_HPP_
