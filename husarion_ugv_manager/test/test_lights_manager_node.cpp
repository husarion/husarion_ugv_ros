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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include <husarion_ugv_manager/lights_manager_node.hpp>
#include "husarion_ugv_utils/test/ros_test_utils.hpp"

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class LightsManagerNodeWrapper : public husarion_ugv_manager::LightsManagerNode
{
public:
  LightsManagerNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : LightsManagerNode(node_name, options)
  {
  }

  ~LightsManagerNodeWrapper() {}

  void RegisterBehaviorTree() { return LightsManagerNode::RegisterBehaviorTree(); }
  bool SystemReady() { return LightsManagerNode::SystemReady(); }

  BT::Blackboard::Ptr GetLightsTreeBlackboard()
  {
    return this->lights_tree_manager_->GetBlackboard();
  }
};

class TestLightsManagerNode : public testing::Test
{
public:
  TestLightsManagerNode();
  ~TestLightsManagerNode();

protected:
  void CreateBTProjectFile(const std::string & tree_xml);
  std::vector<rclcpp::Parameter> CreateTestParameters() const;

  std::shared_ptr<LightsManagerNodeWrapper> lights_manager_node_;

private:
  std::string bt_project_path_;

  const std::string simple_tree_ = R"(
    <root BTCPP_format="4" project_name="Test">
      <BehaviorTree ID="Lights">
        <Sequence>
          <CallSetLedAnimationService id="0" service_name="dummy_service"/>
          <TickAfterTimeout>
            <AlwaysSuccess/>
          </TickAfterTimeout>
        </Sequence>
      </BehaviorTree>
    </root>
  )";
};

TestLightsManagerNode::TestLightsManagerNode()
{
  bt_project_path_ = testing::TempDir() + "test_bt.btproj";

  CreateBTProjectFile(simple_tree_);

  rclcpp::NodeOptions options;
  options.parameter_overrides(CreateTestParameters());

  lights_manager_node_ = std::make_shared<LightsManagerNodeWrapper>("test_lights_manager", options);
  lights_manager_node_->Initialize();
}

TestLightsManagerNode::~TestLightsManagerNode() { std::filesystem::remove(bt_project_path_); }

void TestLightsManagerNode::CreateBTProjectFile(const std::string & tree_xml)
{
  std::ofstream out(bt_project_path_);
  if (out.is_open()) {
    out << tree_xml;
    out.close();
  }
}

std::vector<rclcpp::Parameter> TestLightsManagerNode::CreateTestParameters() const
{
  std::vector<std::string> plugin_libs;
  plugin_libs.push_back("tick_after_timeout_bt_node");

  std::vector<std::string> ros_plugin_libs;
  ros_plugin_libs.push_back("call_set_led_animation_service_bt_node");

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("bt_project_path", bt_project_path_));
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));

  return params;
}

TEST_F(TestLightsManagerNode, SystemReady)
{
  EXPECT_FALSE(lights_manager_node_->SystemReady());

  lights_manager_node_->GetLightsTreeBlackboard()->set<bool>("e_stop_state", true);
  EXPECT_FALSE(lights_manager_node_->SystemReady());

  lights_manager_node_->GetLightsTreeBlackboard()->set<unsigned>("battery_status", 0);
  EXPECT_TRUE(lights_manager_node_->SystemReady());
}

TEST_F(TestLightsManagerNode, BatteryCBBlackboardUpdate)
{
  const float expected_percentage = 0.5;
  const auto expected_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
  const auto expected_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;

  auto battery_state = sensor_msgs::msg::BatteryState();
  battery_state.percentage = expected_percentage;
  battery_state.power_supply_status = expected_status;
  battery_state.power_supply_health = expected_health;

  husarion_ugv_utils::test_utils::PublishAndSpin(
    lights_manager_node_, "battery/battery_status", battery_state);

  auto blackboard = lights_manager_node_->GetLightsTreeBlackboard();
  EXPECT_FLOAT_EQ(blackboard->get<float>("battery_percent"), expected_percentage);
  EXPECT_EQ(blackboard->get<unsigned>("battery_status"), expected_status);
  EXPECT_EQ(blackboard->get<unsigned>("battery_health"), expected_health);
}

TEST_F(TestLightsManagerNode, EStopCBBlackboardUpdate)
{
  const bool expected_state = true;

  auto bool_msg = std_msgs::msg::Bool();
  bool_msg.data = expected_state;

  husarion_ugv_utils::test_utils::PublishAndSpin(
    lights_manager_node_, "hardware/e_stop", bool_msg,
    rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

  auto blackboard = lights_manager_node_->GetLightsTreeBlackboard();
  EXPECT_FLOAT_EQ(blackboard->get<float>("e_stop_state"), expected_state);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
