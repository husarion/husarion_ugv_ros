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

#include "husarion_ugv_battery/battery_publisher/dual_battery_publisher.hpp"

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <utility>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/battery_publisher/battery_publisher.hpp"
#include "husarion_ugv_utils/ros_utils.hpp"

namespace husarion_ugv_battery
{

DualBatteryPublisher::DualBatteryPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
  const double battery_timeout, const std::shared_ptr<Battery> & battery_1,
  const std::shared_ptr<Battery> & battery_2)
: BatteryPublisher(std::move(node), std::move(diagnostic_updater), battery_timeout),
  battery_1_(std::move(battery_1)),
  battery_2_(std::move(battery_2))
{
  battery_pub_ = node->create_publisher<BatteryStateMsg>("battery/battery_status", 5);
  battery_1_pub_ = node->create_publisher<BatteryStateMsg>("_battery/battery_1_status_raw", 5);
  battery_2_pub_ = node->create_publisher<BatteryStateMsg>("_battery/battery_2_status_raw", 5);
  charging_status_pub_ = node->create_publisher<ChargingStatusMsg>("battery/charging_status", 5);
}

void DualBatteryPublisher::Update()
{
  const auto header_stamp = this->GetClock()->now();
  battery_1_->Update(header_stamp, ChargerConnected());
  battery_2_->Update(header_stamp, ChargerConnected());
}

void DualBatteryPublisher::Reset()
{
  const auto header_stamp = this->GetClock()->now();
  battery_1_->Reset(header_stamp);
  battery_2_->Reset(header_stamp);
}

void DualBatteryPublisher::PublishBatteryState()
{
  const auto battery_msg = MergeBatteryMsgs(
    battery_1_->GetBatteryMsg(), battery_2_->GetBatteryMsg());
  battery_pub_->publish(battery_msg);

  battery_1_pub_->publish(battery_1_->GetBatteryMsgRaw());
  battery_2_pub_->publish(battery_2_->GetBatteryMsgRaw());

  BatteryStatusLogger(battery_msg);
}

void DualBatteryPublisher::PublishChargingStatus()
{
  auto const charging_status_msg = MergeChargingStatusMsgs(
    battery_1_->GetChargingStatus(), battery_2_->GetChargingStatus());

  charging_status_pub_->publish(charging_status_msg);
}

void DualBatteryPublisher::LogErrors()
{
  if (battery_1_->HasErrorMsg()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      GetLogger(), *GetClock(), 10000, "Battery nr 1 error: " << battery_1_->GetErrorMsg());
  }
  if (battery_2_->HasErrorMsg()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      GetLogger(), *GetClock(), 10000, "Battery nr 2 error: " << battery_2_->GetErrorMsg());
  }
}

BatteryStateMsg DualBatteryPublisher::MergeBatteryMsgs(
  const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2)
{
  BatteryStateMsg battery_msg;

  battery_msg.header.stamp = battery_msg_1.header.stamp;
  battery_msg.power_supply_technology = battery_msg_1.power_supply_technology;
  battery_msg.cell_voltage = battery_msg_1.cell_voltage;
  battery_msg.cell_temperature = battery_msg_1.cell_temperature;
  battery_msg.location = battery_msg_1.location;
  battery_msg.present = battery_msg_1.present;
  battery_msg.power_supply_status = MergeBatteryPowerSupplyStatus(battery_msg_1, battery_msg_2);

  battery_msg.voltage = (battery_msg_1.voltage + battery_msg_2.voltage) / 2.0f;
  battery_msg.temperature = (battery_msg_1.temperature + battery_msg_2.temperature) / 2.0f;
  battery_msg.current = battery_msg_1.current + battery_msg_2.current;
  battery_msg.percentage = (battery_msg_1.percentage + battery_msg_2.percentage) / 2.0f;
  battery_msg.capacity = battery_msg_1.capacity + battery_msg_2.capacity;
  battery_msg.design_capacity = battery_msg_1.design_capacity + battery_msg_2.design_capacity;
  battery_msg.charge = battery_msg_1.charge + battery_msg_2.charge;

  MergeBatteryPowerSupplyHealth(battery_msg, battery_msg_1, battery_msg_2);

  return battery_msg;
}

std::uint8_t DualBatteryPublisher::MergeBatteryPowerSupplyStatus(
  const BatteryStateMsg & battery_msg_1, const BatteryStateMsg & battery_msg_2) const
{
  if (battery_msg_1.power_supply_status == battery_msg_2.power_supply_status) {
    return battery_msg_1.power_supply_status;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_NOT_CHARGING;
  } else if (
    battery_msg_1.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING ||
    battery_msg_2.power_supply_status == BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING) {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_CHARGING;
  } else {
    return BatteryStateMsg::POWER_SUPPLY_STATUS_UNKNOWN;
  }
}

void DualBatteryPublisher::MergeBatteryPowerSupplyHealth(
  BatteryStateMsg & battery_msg, const BatteryStateMsg & battery_msg_1,
  const BatteryStateMsg & battery_msg_2)
{
  if (battery_msg_1.power_supply_health == battery_msg_2.power_supply_health) {
    battery_msg.power_supply_health = battery_msg_1.power_supply_health;
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_DEAD;
    battery_msg.voltage = std::min<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERHEAT;
    battery_msg.temperature = std::max<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    battery_msg.voltage = std::max<float>(battery_msg_1.voltage, battery_msg_2.voltage);
  } else if (
    battery_msg_1.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD ||
    battery_msg_2.power_supply_health == BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD) {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_COLD;
    battery_msg.temperature = std::min<float>(battery_msg_1.temperature, battery_msg_2.temperature);
  } else {
    battery_msg.power_supply_health = BatteryStateMsg::POWER_SUPPLY_HEALTH_UNKNOWN;
  }
}

ChargingStatusMsg DualBatteryPublisher::MergeChargingStatusMsgs(
  const ChargingStatusMsg & charging_status_msg_1, const ChargingStatusMsg & charging_status_msg_2)
{
  ChargingStatusMsg charging_status_msg;

  try {
    husarion_ugv_utils::ros::VerifyTimestampGap(
      charging_status_msg_1.header, charging_status_msg_2.header, std::chrono::seconds(1));

    charging_status_msg.header = husarion_ugv_utils::ros::MergeHeaders(
      charging_status_msg_1.header, charging_status_msg_2.header);
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      GetLogger(), *GetClock(), 10000,
      "An exception occurred while merging charging status headers: " << e.what());
  }

  charging_status_msg.current = charging_status_msg_1.current + charging_status_msg_2.current;
  charging_status_msg.current_battery_1 = charging_status_msg_1.current;
  charging_status_msg.current_battery_2 = charging_status_msg_2.current;

  if (charging_status_msg_1.charging != charging_status_msg_2.charging) {
    RCLCPP_ERROR_THROTTLE(
      GetLogger(), *GetClock(), 10000, "Charging status mismatch between batteries!");
    charging_status_msg.charging = false;

  } else {
    charging_status_msg.charging = charging_status_msg_1.charging;
  }

  if (charging_status_msg_1.charger_type != charging_status_msg_2.charger_type) {
    RCLCPP_ERROR_THROTTLE(
      GetLogger(), *GetClock(), 10000, "Charger type mismatch between batteries!");
    charging_status_msg.charger_type = ChargingStatusMsg::UNKNOWN;
  } else {
    charging_status_msg.charger_type = charging_status_msg_1.charger_type;
  }

  return charging_status_msg;
}

void DualBatteryPublisher::DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Battery has no errors"};

  if (battery_1_->HasErrorMsg() || battery_2_->HasErrorMsg()) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Battery has error";

    if (battery_1_->HasErrorMsg()) {
      status.add("Error message: battery 1", battery_1_->GetErrorMsg());
    }

    if (battery_2_->HasErrorMsg()) {
      status.add("Error message: battery 2", battery_2_->GetErrorMsg());
    }
  }

  status.summary(error_level, message);
}

void DualBatteryPublisher::DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto battery_1_msg = battery_1_->GetBatteryMsg();
  const auto battery_2_msg = battery_2_->GetBatteryMsg();
  auto charging_status_bat_1 = MapPowerSupplyStatusToString(battery_1_msg.power_supply_status);
  auto charging_status_bat_2 = MapPowerSupplyStatusToString(battery_2_msg.power_supply_status);

  std::string charging_status;
  charging_status_bat_1 == charging_status_bat_2
    ? charging_status = charging_status_bat_1
    : charging_status = "Power supply status not determined, check batteries.";
  status.add("Power supply status", charging_status);

  const auto load_current_bat_1 = battery_1_->GetLoadCurrent();
  const auto load_current_bat_2 = battery_2_->GetLoadCurrent();
  const auto load_current = load_current_bat_1 + load_current_bat_2;
  status.add("Load current total (A)", load_current);
  status.add("Load current battery 1 (A)", load_current_bat_1);
  status.add("Load current battery 2 (A)", load_current_bat_2);

  status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Battery status monitoring");
}

}  // namespace husarion_ugv_battery
