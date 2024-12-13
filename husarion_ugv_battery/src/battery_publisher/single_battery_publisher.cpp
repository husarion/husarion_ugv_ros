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

#include "husarion_ugv_battery/battery_publisher/single_battery_publisher.hpp"

#include <memory>
#include <stdexcept>
#include <utility>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/battery_state.hpp"

#include "husarion_ugv_battery/battery/battery.hpp"
#include "husarion_ugv_battery/battery_publisher/battery_publisher.hpp"

namespace husarion_ugv_battery
{

SingleBatteryPublisher::SingleBatteryPublisher(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
  const double battery_timeout, const std::shared_ptr<Battery> & battery)
: BatteryPublisher(std::move(node), std::move(diagnostic_updater), battery_timeout),
  battery_(std::move(battery))
{
  battery_pub_ = node->create_publisher<BatteryStateMsg>("battery/battery_status", 5);
  battery_1_pub_ = node->create_publisher<BatteryStateMsg>("_battery/battery_1_status_raw", 5);
  charging_status_pub_ = node->create_publisher<ChargingStatusMsg>("battery/charging_status", 5);
}

void SingleBatteryPublisher::Update()
{
  const auto header_stamp = this->GetClock()->now();
  battery_->Update(header_stamp, ChargerConnected());
}

void SingleBatteryPublisher::Reset()
{
  const auto header_stamp = this->GetClock()->now();
  battery_->Reset(header_stamp);
}

void SingleBatteryPublisher::PublishBatteryState()
{
  const auto battery_msg = battery_->GetBatteryMsg();
  battery_pub_->publish(battery_msg);
  battery_1_pub_->publish(battery_->GetBatteryMsgRaw());
  BatteryStatusLogger(battery_msg);
}

void SingleBatteryPublisher::PublishChargingStatus()
{
  auto charging_status_msg = battery_->GetChargingStatus();

  charging_status_msg.current_battery_1 = charging_status_msg.current;
  charging_status_msg.current_battery_2 = std::numeric_limits<float>::quiet_NaN();

  charging_status_pub_->publish(charging_status_msg);
}

void SingleBatteryPublisher::LogErrors()
{
  if (battery_->HasErrorMsg()) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      GetLogger(), *GetClock(), 10000, "Battery error: " << battery_->GetErrorMsg());
  }
}

void SingleBatteryPublisher::DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Battery has no errors"};

  if (battery_->HasErrorMsg()) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Battery has error";

    status.add("Error message", battery_->GetErrorMsg());
  }

  status.summary(error_level, message);
}

void SingleBatteryPublisher::DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  const auto battery_msg = battery_->GetBatteryMsg();

  auto charging_status = MapPowerSupplyStatusToString(battery_msg.power_supply_status);
  status.add("Power supply status", charging_status);

  const auto load_current = battery_->GetLoadCurrent();
  status.add("Load current (A)", load_current);

  status.summary(diagnostic_updater::DiagnosticStatusWrapper::OK, "Battery status monitoring");
}

}  // namespace husarion_ugv_battery
