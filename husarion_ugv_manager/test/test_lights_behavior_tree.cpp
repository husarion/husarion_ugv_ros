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
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "behaviortree_cpp/basic_types.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/bool.hpp"

#include "husarion_ugv_msgs/msg/led_animation.hpp"
#include "husarion_ugv_msgs/srv/set_led_animation.hpp"

#include <husarion_ugv_manager/lights_manager_node.hpp>
#include <utils/behavior_tree_test_utils.hpp>

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using SetLEDAnimationSrv = husarion_ugv_msgs::srv::SetLEDAnimation;
using LEDAnimationMsg = husarion_ugv_msgs::msg::LEDAnimation;

class LightsManagerNodeWrapper : public husarion_ugv_manager::LightsManagerNode
{
public:
  LightsManagerNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : LightsManagerNode(node_name, options)
  {
  }

  ~LightsManagerNodeWrapper() {}

  BT::NodeStatus GetTreeStatus() { return this->lights_tree_manager_->GetTreeStatus(); }
};

class TestLightsBehaviorTree : public testing::Test
{
public:
  TestLightsBehaviorTree();
  ~TestLightsBehaviorTree() {}

protected:
  std::vector<rclcpp::Parameter> CreateTestParameters() const;
  bool SpinWhileRunning(
    const std::chrono::milliseconds & timeout = std::chrono::milliseconds(3000));
  void PublishEStop(const bool data);
  void PublishBatteryState(
    const std::uint8_t status, const std::uint8_t health, const float percentage);

  static constexpr float kLowBatteryAnimPeriod = 1.0;
  static constexpr float kCriticalBatteryAnimPeriod = 0.5;
  static constexpr float kBatteryMaxPercent = 1.0;
  static constexpr float kBatteryLowPercent = 0.39;
  static constexpr float kBatteryCriticalPercent = 0.09;
  static constexpr float kBatteryOptimalPercent = 0.8;

  static constexpr std::array<uint16_t, 2> error_animations_ = {
    LEDAnimationMsg::ERROR,
    LEDAnimationMsg::NO_ERROR,
  };
  static constexpr std::array<uint16_t, 5> battery_animations_ = {
    LEDAnimationMsg::BATTERY_CHARGED,  LEDAnimationMsg::BATTERY_NOMINAL,
    LEDAnimationMsg::CHARGING_BATTERY, LEDAnimationMsg::LOW_BATTERY,
    LEDAnimationMsg::CRITICAL_BATTERY,
  };

  std::size_t current_anim_id_ = -1;
  std::size_t current_error_anim_id = -1;
  std::size_t current_battery_anim_id = -1;

  std::shared_ptr<LightsManagerNodeWrapper> lights_manager_node_;

private:
  void SetLEDAnimationCB(
    const SetLEDAnimationSrv::Request::SharedPtr & request,
    SetLEDAnimationSrv::Response::SharedPtr response);

  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_pub_;
  rclcpp::Publisher<BatteryStateMsg>::SharedPtr battery_state_pub_;
  rclcpp::Service<SetLEDAnimationSrv>::SharedPtr set_led_animation_server_;
};

TestLightsBehaviorTree::TestLightsBehaviorTree()
{
  using namespace std::placeholders;

  rclcpp::NodeOptions options;
  options.parameter_overrides(CreateTestParameters());

  lights_manager_node_ = std::make_shared<LightsManagerNodeWrapper>("test_lights_manager", options);

  e_stop_pub_ = lights_manager_node_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  battery_state_pub_ = lights_manager_node_->create_publisher<BatteryStateMsg>(
    "battery/battery_status", 3);
  set_led_animation_server_ = lights_manager_node_->create_service<SetLEDAnimationSrv>(
    "lights/set_animation", std::bind(&TestLightsBehaviorTree::SetLEDAnimationCB, this, _1, _2));

  lights_manager_node_->Initialize();
}

std::vector<rclcpp::Parameter> TestLightsBehaviorTree::CreateTestParameters() const
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("husarion_ugv_manager");
  const std::string bt_project_path = panther_manager_pkg_path + "/behavior_trees/LightsBT.btproj";

  std::vector<std::string> plugin_libs;
  plugin_libs.push_back("tick_after_timeout_bt_node");

  std::vector<std::string> ros_plugin_libs;
  ros_plugin_libs.push_back("call_set_led_animation_service_bt_node");

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("bt_project_path", bt_project_path));
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));
  params.push_back(rclcpp::Parameter("battery.anim_period.low", kLowBatteryAnimPeriod));
  params.push_back(rclcpp::Parameter("battery.anim_period.critical", kCriticalBatteryAnimPeriod));

  return params;
}

bool TestLightsBehaviorTree::SpinWhileRunning(const std::chrono::milliseconds & timeout)
{
  return behavior_tree::test_utils::SpinWhileRunning(
    lights_manager_node_, [&]() { return lights_manager_node_->GetTreeStatus(); }, timeout);
}

void TestLightsBehaviorTree::PublishEStop(const bool data)
{
  BoolMsg msg;
  msg.data = data;
  e_stop_pub_->publish(msg);
}

void TestLightsBehaviorTree::PublishBatteryState(
  const std::uint8_t status, const std::uint8_t health, const float percentage)
{
  BatteryStateMsg msg;
  msg.power_supply_status = status;
  msg.power_supply_health = health;
  msg.percentage = percentage;
  battery_state_pub_->publish(msg);
}

void TestLightsBehaviorTree::SetLEDAnimationCB(
  const SetLEDAnimationSrv::Request::SharedPtr & request,
  SetLEDAnimationSrv::Response::SharedPtr response)
{
  const auto anim_id = request->animation.id;
  if (
    std::find(error_animations_.begin(), error_animations_.end(), anim_id) !=
    error_animations_.end()) {
    current_error_anim_id = anim_id;
  } else if (
    std::find(battery_animations_.begin(), battery_animations_.end(), anim_id) !=
    battery_animations_.end()) {
    current_battery_anim_id = anim_id;
  } else {
    current_anim_id_ = request->animation.id;
  }

  response->success = true;
}

TEST_F(TestLightsBehaviorTree, UnknownBatteryStatus)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryMaxPercent);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_error_anim_id, LEDAnimationMsg::ERROR);
}

TEST_F(TestLightsBehaviorTree, NotCharging)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryMaxPercent);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_anim_id_, LEDAnimationMsg::READY);
}

TEST_F(TestLightsBehaviorTree, Ready)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryMaxPercent);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_anim_id_, LEDAnimationMsg::READY);
}

TEST_F(TestLightsBehaviorTree, EStop)
{
  PublishEStop(true);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryMaxPercent);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_anim_id_, LEDAnimationMsg::E_STOP);
}

TEST_F(TestLightsBehaviorTree, LowBattery)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryLowPercent);

  // Wait for the low battery animation period to pass, then spin
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kLowBatteryAnimPeriod * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_battery_anim_id, LEDAnimationMsg::LOW_BATTERY);
}

TEST_F(TestLightsBehaviorTree, CriticalBattery)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryCriticalPercent);

  // Wait for the critical battery animation period to pass, then spin
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kCriticalBatteryAnimPeriod * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_battery_anim_id, LEDAnimationMsg::CRITICAL_BATTERY);
}

TEST_F(TestLightsBehaviorTree, ChargingOverheat)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT,
    kBatteryMaxPercent);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_EQ(current_error_anim_id, LEDAnimationMsg::ERROR);
}

TEST_F(TestLightsBehaviorTree, Charging)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD,
    kBatteryOptimalPercent);

  ASSERT_TRUE(SpinWhileRunning(std::chrono::milliseconds(3000)));
  EXPECT_EQ(current_battery_anim_id, LEDAnimationMsg::CHARGING_BATTERY);
}

TEST_F(TestLightsBehaviorTree, BatteryCharged)
{
  PublishEStop(false);
  PublishBatteryState(
    BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING, BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD, 1.0);

  ASSERT_TRUE(SpinWhileRunning(std::chrono::milliseconds(3000)));
  EXPECT_EQ(current_battery_anim_id, LEDAnimationMsg::BATTERY_CHARGED);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
