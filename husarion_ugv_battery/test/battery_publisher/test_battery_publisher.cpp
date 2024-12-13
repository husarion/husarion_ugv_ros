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
#include <thread>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_msgs/msg/io_state.hpp"

#include "husarion_ugv_battery/battery_publisher/battery_publisher.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;

class BatteryPublisherWrapper : public husarion_ugv_battery::BatteryPublisher
{
public:
  BatteryPublisherWrapper(
    const rclcpp::Node::SharedPtr & node,
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater)
  : husarion_ugv_battery::BatteryPublisher(node, diagnostic_updater, kBatteryTimeout)
  {
  }

  bool TimeoutReached() { return BatteryPublisher::TimeoutReached(); }

  void BatteryStatusLogger(const BatteryStateMsg & battery_state)
  {
    return BatteryPublisher::BatteryStatusLogger(battery_state);
  }

  bool ChargerConnected() const { return BatteryPublisher::ChargerConnected(); }

  // Mock up methods to make wrapper a valid subclass
  void Update() {};
  void Reset() {};
  void PublishBatteryState() {};
  void PublishChargingStatus() {};
  void LogErrors() {};
  void DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(0, "");  // Avoid unused parameter compiler warning
  };
  void DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
  {
    status.summary(0, "");  // Avoid unused parameter compiler warning
  };

protected:
  static constexpr double kBatteryTimeout = 0.2;
};

class TestBatteryPublisher : public testing::Test
{
public:
  TestBatteryPublisher();
  ~TestBatteryPublisher() {}

protected:
  static constexpr float kBatteryTimeout = 0.5;
  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
  std::shared_ptr<BatteryPublisherWrapper> battery_publisher_;
};

TestBatteryPublisher::TestBatteryPublisher()
{
  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("battery_timeout", kBatteryTimeout));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  node_ = std::make_shared<rclcpp::Node>("node", options);
  diagnostic_updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
  io_state_pub_ = node_->create_publisher<IOStateMsg>(
    "/hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  battery_publisher_ = std::make_shared<BatteryPublisherWrapper>(node_, diagnostic_updater_);
}

TEST_F(TestBatteryPublisher, TimeoutReached)
{
  // Reset battery info time
  battery_publisher_->Publish();
  EXPECT_FALSE(battery_publisher_->TimeoutReached());

  std::this_thread::sleep_for(
    std::chrono::milliseconds(static_cast<int>(kBatteryTimeout * 1000 + 100)));
  EXPECT_TRUE(battery_publisher_->TimeoutReached());
}

TEST_F(TestBatteryPublisher, ChargerConnected)
{
  EXPECT_FALSE(battery_publisher_->ChargerConnected());

  IOStateMsg io_state;
  io_state.charger_connected = true;
  io_state_pub_->publish(io_state);
  rclcpp::spin_some(node_->get_node_base_interface());
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  EXPECT_TRUE(battery_publisher_->ChargerConnected());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
