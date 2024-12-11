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
#include <memory>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_battery/battery/adc_battery.hpp"
#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/battery_publisher/single_battery_publisher.hpp"
#include "husarion_ugv_utils/test/ros_test_utils.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;

class TestSingleBatteryPublisher : public testing::Test
{
public:
  TestSingleBatteryPublisher();
  ~TestSingleBatteryPublisher() {}

protected:
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;

  std::shared_ptr<husarion_ugv_battery::Battery> battery_;
  std::shared_ptr<husarion_ugv_battery::BatteryPublisher> battery_publisher_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;

  static constexpr double kBatteryTimeout = 0.2f;
};

TestSingleBatteryPublisher::TestSingleBatteryPublisher()
{
  husarion_ugv_battery::ADCBatteryParams params = {10, 10, 10, 10};
  battery_ = std::make_shared<husarion_ugv_battery::ADCBattery>(
    [&]() { return 1.6; }, [&]() { return 0.02; }, [&]() { return 1.6; }, [&]() { return 0.4; },
    params);

  node_ = std::make_shared<rclcpp::Node>("node");
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
  battery_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/battery/battery_status", 10,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = node_->create_subscription<BatteryStateMsg>(
    "/_battery/battery_1_status_raw", 10,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_publisher_ = std::make_shared<husarion_ugv_battery::SingleBatteryPublisher>(
    node_, diagnostic_updater_, kBatteryTimeout, battery_);
}

TEST_F(TestSingleBatteryPublisher, CorrectTopicPublished)
{
  battery_publisher_->Publish();
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    node_, battery_state_, std::chrono::milliseconds(1000)));
  battery_publisher_->Publish();
  ASSERT_TRUE(husarion_ugv_utils::test_utils::WaitForMsg(
    node_, battery_1_state_, std::chrono::milliseconds(1000)));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
