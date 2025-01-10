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

#include "husarion_ugv_hardware_interfaces/robot_system/lynx_system.hpp"

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rclcpp/logging.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/lynx_robot_driver.hpp"

#include "husarion_ugv_utils/diagnostics.hpp"

namespace husarion_ugv_hardware_interfaces
{

void LynxSystem::ReadCANopenSettingsDriverCANIDs()
{
  const auto driver_can_id = std::stoi(info_.hardware_parameters["driver_can_id"]);
  canopen_settings_.driver_can_ids.emplace(DriverNames::DEFAULT, driver_can_id);
}

void LynxSystem::DefineRobotDriver()
{
  robot_driver_ = std::make_shared<LynxRobotDriver>(canopen_settings_, drivetrain_settings_);
}

void LynxSystem::UpdateHwStates()
{
  const auto data = robot_driver_->GetData(DriverNames::DEFAULT);

  const auto left = data.GetMotorState(MotorChannels::LEFT);
  const auto right = data.GetMotorState(MotorChannels::RIGHT);

  hw_states_positions_[0] = left.GetPosition();
  hw_states_positions_[1] = right.GetPosition();
  hw_states_positions_[2] = left.GetPosition();
  hw_states_positions_[3] = right.GetPosition();

  hw_states_velocities_[0] = left.GetVelocity();
  hw_states_velocities_[1] = right.GetVelocity();
  hw_states_velocities_[2] = left.GetVelocity();
  hw_states_velocities_[3] = right.GetVelocity();

  hw_states_efforts_[0] = left.GetTorque();
  hw_states_efforts_[1] = right.GetTorque();
  hw_states_efforts_[2] = left.GetTorque();
  hw_states_efforts_[3] = right.GetTorque();
}

void LynxSystem::UpdateMotorsStateDataTimedOut()
{
  if (robot_driver_->GetData(DriverNames::DEFAULT).IsMotorStatesDataTimedOut()) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 1000, "PDO motor state data timeout.");
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, false);
  }
}

void LynxSystem::UpdateDriverStateMsg()
{
  const auto driver_data = robot_driver_->GetData(DriverNames::DEFAULT);

  system_ros_interface_->UpdateMsgDriversStates(DriverNames::DEFAULT, driver_data.GetDriverState());
  system_ros_interface_->UpdateMsgErrorFlags(DriverNames::DEFAULT, driver_data);

  CANErrors can_errors;
  can_errors.error = roboteq_error_filter_->IsError();
  can_errors.write_pdo_cmds_error = roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS);
  can_errors.read_pdo_motor_states_error =
    roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES);
  can_errors.read_pdo_driver_state_error =
    roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);

  DriverCANErrors driver_can_errors;
  driver_can_errors.motor_states_data_timed_out = driver_data.IsMotorStatesDataTimedOut();
  driver_can_errors.driver_state_data_timed_out = driver_data.IsDriverStateDataTimedOut();
  driver_can_errors.can_error = driver_data.IsCANError();
  driver_can_errors.heartbeat_timeout = driver_data.IsHeartbeatTimeout();

  can_errors.driver_errors.emplace(DriverNames::DEFAULT, driver_can_errors);

  system_ros_interface_->UpdateMsgErrors(can_errors);
}

void LynxSystem::UpdateFlagErrors()
{
  const auto driver_data = robot_driver_->GetData(DriverNames::DEFAULT);
  if (driver_data.IsFlagError()) {
    RCLCPP_WARN_STREAM_THROTTLE(
      logger_, steady_clock_, 1000,
      "Driver in error state:\n"
        << driver_data.GetFlagErrorLog());
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);

    HandleRobotDriverWriteOperation([this] { robot_driver_->AttemptErrorFlagReset(); });
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
  }
}

void LynxSystem::UpdateDriverStateDataTimedOut()
{
  if (robot_driver_->GetData(DriverNames::DEFAULT).IsDriverStateDataTimedOut()) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 1000, "PDO driver state timeout.");
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, false);
  }
}

void LynxSystem::DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"No error detected."};

  const auto driver_data = robot_driver_->GetData(DriverNames::DEFAULT);
  if (driver_data.IsError()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Error detected.";

    husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
      status, driver_data.GetErrorMap(), "Driver error: ");
  }

  if (roboteq_error_filter_->IsError()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Error detected.";

    husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
      status, roboteq_error_filter_->GetErrorMap(), "", " error");
  }

  status.summary(level, message);
}

void LynxSystem::DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Panther system status monitoring."};

  const auto driver_state = robot_driver_->GetData(DriverNames::DEFAULT).GetDriverState();

  status.add("Default driver voltage (V)", driver_state.GetVoltage());
  status.add("Default driver current (A)", driver_state.GetCurrent());
  status.add("Default driver temperature (\u00B0C)", driver_state.GetTemperature());
  status.add(
    "Default driver heatsink temperature (\u00B0C)", driver_state.GetHeatsinkTemperature());

  status.summary(level, message);
}

std::vector<float> LynxSystem::GetSpeedCommands() const
{
  return {
    static_cast<float>(hw_commands_velocities_[0]), static_cast<float>(hw_commands_velocities_[1])};
}

}  // namespace husarion_ugv_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  husarion_ugv_hardware_interfaces::LynxSystem, hardware_interface::SystemInterface)
