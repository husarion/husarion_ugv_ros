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

#include "husarion_ugv_battery/battery_driver_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_battery/adc_data_reader.hpp"
#include "husarion_ugv_battery/battery/adc_battery.hpp"
#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/battery/roboteq_battery.hpp"
#include "husarion_ugv_battery/battery_parameters.hpp"
#include "husarion_ugv_battery/battery_publisher/battery_publisher.hpp"
#include "husarion_ugv_battery/battery_publisher/dual_battery_publisher.hpp"
#include "husarion_ugv_battery/battery_publisher/single_battery_publisher.hpp"

namespace husarion_ugv_battery
{

BatteryDriverNode::BatteryDriverNode(
  const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: Node(node_name, ns, options),
  diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->param_listener_ =
    std::make_shared<battery::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  // Running at 10 Hz
  battery_pub_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&BatteryDriverNode::BatteryPubTimerCB, this));

  diagnostic_updater_->setHardwareID("Battery");

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

void BatteryDriverNode::Initialize()
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  try {
    InitializeWithADCBattery();
    return;
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "An exception occurred while initializing with ADC: "
                            << e.what()
                            << " Falling back to using Roboteq drivers to publish battery data.");
  }
  InitializeWithRoboteqBattery();

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void BatteryDriverNode::InitializeWithADCBattery()
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing with ADC data.");

  const std::string adc0_device_path = this->params_.adc.device0;
  const std::string adc1_device_path = this->params_.adc.device1;

  adc0_reader_ = std::make_shared<ADCDataReader>(adc0_device_path);
  adc1_reader_ = std::make_shared<ADCDataReader>(adc1_device_path);

  const ADCBatteryParams battery_params = {
    static_cast<std::size_t>(this->params_.ma_window_len.voltage),
    static_cast<std::size_t>(this->params_.ma_window_len.current),
    static_cast<std::size_t>(this->params_.adc.ma_window_len.temp),
    static_cast<std::size_t>(this->params_.adc.ma_window_len.charge),
  };

  battery_2_ = std::make_shared<ADCBattery>(
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 3, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 1, kADCCurrentOffset),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 0, 0),
    std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 2, 0), battery_params);

  if (battery_2_->Present()) {
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 2, kADCCurrentOffset),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<DualBatteryPublisher>(
      this->shared_from_this(), diagnostic_updater_, params_.battery_timeout, battery_1_,
      battery_2_);
  } else {
    battery_2_.reset();
    battery_1_ = std::make_shared<ADCBattery>(
      std::bind(&ADCDataReader::GetADCMeasurement, *adc1_reader_, 0, 0),
      [&]() {
        return adc1_reader_->GetADCMeasurement(2, kADCCurrentOffset) +
               adc1_reader_->GetADCMeasurement(1, kADCCurrentOffset);
      },
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 1, 0),
      std::bind(&ADCDataReader::GetADCMeasurement, *adc0_reader_, 3, 0), battery_params);
    battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
      this->shared_from_this(), diagnostic_updater_, params_.battery_timeout, battery_1_);
  }

  RCLCPP_INFO(this->get_logger(), "Initialized battery driver using ADC data.");
}

void BatteryDriverNode::InitializeWithRoboteqBattery()
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing with Roboteq data.");

  const RoboteqBatteryParams battery_params = {
    static_cast<float>(this->params_.roboteq.driver_state_timeout),
    static_cast<std::size_t>(this->params_.ma_window_len.voltage),
    static_cast<std::size_t>(this->params_.ma_window_len.current),
  };

  driver_state_sub_ = this->create_subscription<RobotDriverStateMsg>(
    "hardware/robot_driver_state", 5,
    [&](const RobotDriverStateMsg::SharedPtr msg) { driver_state_ = msg; });

  battery_1_ = std::make_shared<RoboteqBattery>([&]() { return driver_state_; }, battery_params);

  battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
    this->shared_from_this(), diagnostic_updater_, this->params_.battery_timeout, battery_1_);

  RCLCPP_INFO(this->get_logger(), "Initialized battery driver using motor controllers data.");
}

void BatteryDriverNode::BatteryPubTimerCB()
{
  if (!battery_publisher_) {
    Initialize();
    return;
  }
  battery_publisher_->Publish();
}

}  // namespace husarion_ugv_battery
