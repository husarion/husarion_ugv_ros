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
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "husarion_ugv_msgs/msg/io_state.hpp"
#include "husarion_ugv_msgs/msg/system_status.hpp"

#include <husarion_ugv_manager/safety_manager_node.hpp>
#include <utils/behavior_tree_test_utils.hpp>

using BoolMsg = std_msgs::msg::Bool;
using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using SystemStatusMsg = husarion_ugv_msgs::msg::SystemStatus;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

class SafetyManagerNodeWrapper : public husarion_ugv_manager::SafetyManagerNode
{
public:
  SafetyManagerNodeWrapper(
    const std::string & node_name, const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : SafetyManagerNode(node_name, options)
  {
  }

  ~SafetyManagerNodeWrapper() {}

  BT::BehaviorTreeFactory & GetFactory() { return this->factory_; }
  BT::NodeStatus GetSafetyTreeStatus() { return this->safety_tree_manager_->GetTreeStatus(); }
  BT::NodeStatus GetShutdownTreeStatus() { return this->shutdown_tree_manager_->GetTreeStatus(); }

  template <typename T>
  void SetSafetyTreeBlackboard(const std::string name, const T value)
  {
    this->safety_tree_manager_->GetBlackboard()->set<T>(name, value);
  }
};

class TestSafetyBehaviorTree : public testing::Test
{
public:
  TestSafetyBehaviorTree();
  ~TestSafetyBehaviorTree();

protected:
  std::vector<rclcpp::Parameter> CreateTestParameters() const;
  bool SpinWhileRunning();
  void SetSafetyBlackboardDefaultStates();

  static constexpr float kFanTurnOffTimeout = 0.01;
  static constexpr float kBatteryOptimalTemp = 22.0;

  bool fan_state_ = false;
  bool aux_power_state_ = true;
  bool e_stop_triggered_ = false;
  std::shared_ptr<SafetyManagerNodeWrapper> safety_manager_node_;

private:
  void FanServerCB(
    const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response);
  void AUXPowerServerCB(
    const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response);
  void EStopTriggerSrvCB(
    const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response);

  rclcpp::Service<SetBoolSrv>::SharedPtr fan_server_;
  rclcpp::Service<SetBoolSrv>::SharedPtr aux_power_server_;
  rclcpp::Service<TriggerSrv>::SharedPtr e_stop_trigger_server_;
};

TestSafetyBehaviorTree::TestSafetyBehaviorTree()
{
  using namespace std::placeholders;

  rclcpp::init(0, nullptr);

  rclcpp::NodeOptions options;
  options.parameter_overrides(CreateTestParameters());

  safety_manager_node_ = std::make_shared<SafetyManagerNodeWrapper>("test_safety_manager", options);

  fan_server_ = safety_manager_node_->create_service<SetBoolSrv>(
    "hardware/fan_enable", std::bind(&TestSafetyBehaviorTree::FanServerCB, this, _1, _2));
  aux_power_server_ = safety_manager_node_->create_service<SetBoolSrv>(
    "hardware/aux_power_enable",
    std::bind(&TestSafetyBehaviorTree::AUXPowerServerCB, this, _1, _2));
  e_stop_trigger_server_ = safety_manager_node_->create_service<TriggerSrv>(
    "hardware/e_stop_trigger", std::bind(&TestSafetyBehaviorTree::EStopTriggerSrvCB, this, _1, _2));

  // Replace shutdown bt nodes to avoid turning off the test devices
  safety_manager_node_->GetFactory().addSubstitutionRule("ExecuteCommand", "AlwaysSuccess");
  safety_manager_node_->GetFactory().addSubstitutionRule("ShutdownHostsFromFile", "AlwaysSuccess");
  safety_manager_node_->Initialize();
}

TestSafetyBehaviorTree::~TestSafetyBehaviorTree() { rclcpp::shutdown(); }

std::vector<rclcpp::Parameter> TestSafetyBehaviorTree::CreateTestParameters() const
{
  const auto panther_manager_pkg_path =
    ament_index_cpp::get_package_share_directory("husarion_ugv_manager");
  const std::string bt_project_path = panther_manager_pkg_path + "/behavior_trees/SafetyBT.btproj";

  std::vector<std::string> plugin_libs;
  plugin_libs.push_back("tick_after_timeout_bt_node");
  plugin_libs.push_back("execute_command_bt_node");
  plugin_libs.push_back("shutdown_hosts_from_file_bt_node");
  plugin_libs.push_back("signal_shutdown_bt_node");

  std::vector<std::string> ros_plugin_libs;
  ros_plugin_libs.push_back("call_set_bool_service_bt_node");
  ros_plugin_libs.push_back("call_trigger_service_bt_node");

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("bt_project_path", bt_project_path));
  params.push_back(rclcpp::Parameter("plugin_libs", plugin_libs));
  params.push_back(rclcpp::Parameter("ros_plugin_libs", ros_plugin_libs));
  params.push_back(rclcpp::Parameter("fan_turn_off_timeout", kFanTurnOffTimeout));

  return params;
}

bool TestSafetyBehaviorTree::SpinWhileRunning()
{
  return behavior_tree::test_utils::SpinWhileRunning(
    safety_manager_node_, [&]() { return safety_manager_node_->GetSafetyTreeStatus(); },
    std::chrono::milliseconds(1000));
}

void TestSafetyBehaviorTree::SetSafetyBlackboardDefaultStates()
{
  safety_manager_node_->SetSafetyTreeBlackboard("e_stop_state", false);
  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_status", BatteryStateMsg::POWER_SUPPLY_STATUS_DISCHARGING);
  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);
  safety_manager_node_->SetSafetyTreeBlackboard("bat_temp", kBatteryOptimalTemp);
  safety_manager_node_->SetSafetyTreeBlackboard("aux_state", true);
  safety_manager_node_->SetSafetyTreeBlackboard("fan_state", false);
  safety_manager_node_->SetSafetyTreeBlackboard("cpu_temp", 0.0f);
  safety_manager_node_->SetSafetyTreeBlackboard("driver_temp", 0.0f);
}

void TestSafetyBehaviorTree::FanServerCB(
  const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response)
{
  fan_state_ = request->data;
  safety_manager_node_->SetSafetyTreeBlackboard("fan_state", fan_state_);
  response->success = true;
}

void TestSafetyBehaviorTree::AUXPowerServerCB(
  const SetBoolSrv::Request::SharedPtr & request, SetBoolSrv::Response::SharedPtr response)
{
  aux_power_state_ = request->data;
  safety_manager_node_->SetSafetyTreeBlackboard("aux_state", aux_power_state_);
  response->success = true;
}

void TestSafetyBehaviorTree::EStopTriggerSrvCB(
  const TriggerSrv::Request::SharedPtr & /*request*/, TriggerSrv::Response::SharedPtr response)
{
  e_stop_triggered_ = true;
  safety_manager_node_->SetSafetyTreeBlackboard("e_stop_state", true);
  response->success = true;
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanAtStartup)
{
  const float fan_turn_off_timeout = 1.0;

  safety_manager_node_->SetSafetyTreeBlackboard("FAN_TURN_OFF_TIMEOUT", fan_turn_off_timeout);
  SetSafetyBlackboardDefaultStates();

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(fan_state_);

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(fan_turn_off_timeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanCpuTemp)
{
  const float high_cpu_temp = 70.1;
  const float edge_cpu_temp = 59.9;

  SetSafetyBlackboardDefaultStates();

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard("cpu_temp", high_cpu_temp);
  ASSERT_TRUE(SpinWhileRunning());

  EXPECT_TRUE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard("cpu_temp", edge_cpu_temp);
  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, TurnOnFanDriverTemp)
{
  const float high_driver_temp = 45.1;
  const float edge_driver_temp = 34.9;

  SetSafetyBlackboardDefaultStates();

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard("driver_temp", high_driver_temp);
  ASSERT_TRUE(SpinWhileRunning());

  EXPECT_TRUE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard("driver_temp", edge_driver_temp);
  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOvervoltage)
{
  SetSafetyBlackboardDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(e_stop_triggered_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthDead)
{
  SetSafetyBlackboardDefaultStates();
  ASSERT_TRUE(SpinWhileRunning());

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD);

  // Don't check if SpinWhile Running returns true - safety tree will be halted and may not return
  // success.
  SpinWhileRunning();
  // Shutdown of the robot should be invoked so the shutdown tree status should change
  EXPECT_NE(BT::NodeStatus::IDLE, safety_manager_node_->GetShutdownTreeStatus());
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatTurnOnFan)
{
  SetSafetyBlackboardDefaultStates();

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_FALSE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_TRUE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_GOOD);

  // check if change of power supply health turns off the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatCriticalTemp)
{
  const float critical_bat_temp = 55.1;

  SetSafetyBlackboardDefaultStates();

  // wait for automatic turn off of the fan
  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<unsigned>(kFanTurnOffTimeout * 1000)));
  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_FALSE(e_stop_triggered_);
  EXPECT_TRUE(aux_power_state_);
  EXPECT_FALSE(fan_state_);

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT);
  safety_manager_node_->SetSafetyTreeBlackboard("bat_temp", critical_bat_temp);

  ASSERT_TRUE(SpinWhileRunning());
  EXPECT_TRUE(e_stop_triggered_);
  EXPECT_FALSE(aux_power_state_);
  EXPECT_TRUE(fan_state_);
}

TEST_F(TestSafetyBehaviorTree, PowerSupplyHealthOverheatFatalTemp)
{
  const float fatal_bat_temp = 62.1;

  SetSafetyBlackboardDefaultStates();

  safety_manager_node_->SetSafetyTreeBlackboard(
    "battery_health", BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT);
  safety_manager_node_->SetSafetyTreeBlackboard("bat_temp", fatal_bat_temp);

  // Don't check if SpinWhile Running returns true - safety tree will be halted and may not return
  // success.
  SpinWhileRunning();
  // Shutdown of the robot should be invoked so the shutdown tree status should change
  EXPECT_NE(BT::NodeStatus::IDLE, safety_manager_node_->GetShutdownTreeStatus());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
