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

#ifndef HUSARION_UGV_BATTERY_UTILS_TEST_BATTERY_DRIVER_NODE_HPP_
#define HUSARION_UGV_BATTERY_UTILS_TEST_BATTERY_DRIVER_NODE_HPP_

#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_msgs/msg/io_state.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"

#include "husarion_ugv_battery/battery_driver_node.hpp"

using BatteryStateMsg = sensor_msgs::msg::BatteryState;
using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;

class TestBatteryNode : public testing::Test
{
public:
  TestBatteryNode(const bool use_adc_battery = true, const bool dual_battery = false);
  ~TestBatteryNode();

protected:
  template <typename T>
  void WriteNumberToFile(const T number, const std::string & file_path);

  static constexpr char kADCDevice0[] = "iio:device0";
  static constexpr char kADCDevice1[] = "iio:device1";

  std::filesystem::path device0_path_;
  std::filesystem::path device1_path_;
  BatteryStateMsg::SharedPtr battery_state_;
  BatteryStateMsg::SharedPtr battery_1_state_;
  BatteryStateMsg::SharedPtr battery_2_state_;
  std::shared_ptr<husarion_ugv_battery::BatteryDriverNode> battery_driver_node_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_1_sub_;
  rclcpp::Subscription<BatteryStateMsg>::SharedPtr battery_2_sub_;
  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_pub_;
  rclcpp::Publisher<RobotDriverStateMsg>::SharedPtr driver_state_pub_;
};

TestBatteryNode::TestBatteryNode(const bool use_adc_battery, const bool dual_battery)
{
  std::vector<rclcpp::Parameter> params;

  if (use_adc_battery) {
    device0_path_ = std::filesystem::path(testing::TempDir()) / kADCDevice0;
    device1_path_ = std::filesystem::path(testing::TempDir()) / kADCDevice1;

    params.push_back(rclcpp::Parameter("adc.device0", device0_path_.string()));
    params.push_back(rclcpp::Parameter("adc.device1", device1_path_.string()));

    // Create the device0 and device1 directories if they do not exist
    std::filesystem::create_directory(device0_path_);
    std::filesystem::create_directory(device1_path_);

    // Create only files that are required for adc_node to start
    int dual_bat_volt = dual_battery ? 800 : 1600;
    WriteNumberToFile<int>(dual_bat_volt, std::filesystem::path(device0_path_ / "in_voltage0_raw"));
    WriteNumberToFile<int>(800, std::filesystem::path(device0_path_ / "in_voltage1_raw"));
    WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage2_raw"));
    WriteNumberToFile<int>(2, std::filesystem::path(device0_path_ / "in_voltage3_raw"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage0_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage1_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage2_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device0_path_ / "in_voltage3_scale"));

    WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage0_raw"));
    WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage1_raw"));
    WriteNumberToFile<int>(600, std::filesystem::path(device1_path_ / "in_voltage2_raw"));
    WriteNumberToFile<int>(1400, std::filesystem::path(device1_path_ / "in_voltage3_raw"));
    WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage0_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage1_scale"));
    WriteNumberToFile<float>(2.0, std::filesystem::path(device1_path_ / "in_voltage2_scale"));
    WriteNumberToFile<float>(1.0, std::filesystem::path(device1_path_ / "in_voltage3_scale"));
  }

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  battery_driver_node_ = std::make_shared<husarion_ugv_battery::BatteryDriverNode>(
    "battery_driver", "/test_battery", options);

  battery_sub_ = battery_driver_node_->create_subscription<BatteryStateMsg>(
    "battery/battery_status", 10,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_state_ = msg; });
  battery_1_sub_ = battery_driver_node_->create_subscription<BatteryStateMsg>(
    "_battery/battery_1_status_raw", 10,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_1_state_ = msg; });
  battery_2_sub_ = battery_driver_node_->create_subscription<BatteryStateMsg>(
    "_battery/battery_2_status_raw", 10,
    [&](const BatteryStateMsg::SharedPtr msg) { battery_2_state_ = msg; });

  io_state_pub_ = battery_driver_node_->create_publisher<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  driver_state_pub_ = battery_driver_node_->create_publisher<RobotDriverStateMsg>(
    "hardware/robot_driver_state", 10);
}

TestBatteryNode::~TestBatteryNode()
{
  // Delete the devices directories
  if (std::filesystem::exists(device0_path_)) {
    std::filesystem::remove_all(device0_path_);
  }
  if (std::filesystem::exists(device1_path_)) {
    std::filesystem::remove_all(device1_path_);
  }
}

template <typename T>
void TestBatteryNode::WriteNumberToFile(const T number, const std::string & file_path)
{
  std::ofstream file(file_path);
  if (!file) {
    throw std::runtime_error("Failed to create file: " + file_path);
  }

  file << number;
  if (!file) {
    throw std::runtime_error("Failed to write to file: " + file_path);
  }
}

#endif  // HUSARION_UGV_BATTERY_UTILS_TEST_BATTERY_DRIVER_NODE_HPP_
