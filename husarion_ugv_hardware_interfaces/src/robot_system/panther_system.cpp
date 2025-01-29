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

#include "husarion_ugv_hardware_interfaces/robot_system/panther_system.hpp"

#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_status_wrapper.hpp"
#include "rclcpp/logging.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/panther_robot_driver.hpp"

#include "husarion_ugv_utils/diagnostics.hpp"

namespace husarion_ugv_hardware_interfaces
{

void PantherSystem::ReadCANopenSettingsDriverCANIDs()
{
  const auto front_driver_can_id = std::stoi(info_.hardware_parameters["front_driver_can_id"]);
  const auto rear_driver_can_id = std::stoi(info_.hardware_parameters["rear_driver_can_id"]);
  canopen_settings_.driver_can_ids.emplace(DriverNames::FRONT, front_driver_can_id);
  canopen_settings_.driver_can_ids.emplace(DriverNames::REAR, rear_driver_can_id);
}

void PantherSystem::DefineRobotDriver()
{
  robot_driver_ = std::make_shared<PantherRobotDriver>(canopen_settings_, drivetrain_settings_);
}

void PantherSystem::UpdateHwStates()
{
  const auto front_data = robot_driver_->GetData(DriverNames::FRONT);
  const auto rear_data = robot_driver_->GetData(DriverNames::REAR);

  const auto fl_motor_state = front_data.GetMotorState(MotorChannels::LEFT);
  const auto fr_motor_state = front_data.GetMotorState(MotorChannels::RIGHT);
  const auto rl_motor_state = rear_data.GetMotorState(MotorChannels::LEFT);
  const auto rr_motor_state = rear_data.GetMotorState(MotorChannels::RIGHT);

  hw_states_positions_[0] = fl_motor_state.GetPosition();
  hw_states_positions_[1] = fr_motor_state.GetPosition();
  hw_states_positions_[2] = rl_motor_state.GetPosition();
  hw_states_positions_[3] = rr_motor_state.GetPosition();

  hw_states_velocities_[0] = fl_motor_state.GetVelocity();
  hw_states_velocities_[1] = fr_motor_state.GetVelocity();
  hw_states_velocities_[2] = rl_motor_state.GetVelocity();
  hw_states_velocities_[3] = rr_motor_state.GetVelocity();

  hw_states_efforts_[0] = fl_motor_state.GetTorque();
  hw_states_efforts_[1] = fr_motor_state.GetTorque();
  hw_states_efforts_[2] = rl_motor_state.GetTorque();
  hw_states_efforts_[3] = rr_motor_state.GetTorque();
}

void PantherSystem::UpdateMotorsStateDataTimedOut()
{
  if (
    robot_driver_->GetData(DriverNames::FRONT).IsMotorStatesDataTimedOut() ||
    robot_driver_->GetData(DriverNames::REAR).IsMotorStatesDataTimedOut()) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 1000, "PDO motor state data timeout.");
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, false);
  }
}

void PantherSystem::UpdateDriverStateMsg()
{
  const auto front_driver_data = robot_driver_->GetData(DriverNames::FRONT);
  const auto rear_driver_data = robot_driver_->GetData(DriverNames::REAR);

  system_ros_interface_->UpdateMsgDriversStates(
    DriverNames::FRONT, front_driver_data.GetDriverState());
  system_ros_interface_->UpdateMsgDriversStates(
    DriverNames::REAR, rear_driver_data.GetDriverState());
  system_ros_interface_->UpdateMsgErrorFlags(DriverNames::FRONT, front_driver_data);
  system_ros_interface_->UpdateMsgErrorFlags(DriverNames::REAR, rear_driver_data);

  CANErrors can_errors;
  can_errors.error = roboteq_error_filter_->IsError();
  can_errors.write_pdo_cmds_error = roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS);
  can_errors.read_pdo_motor_states_error =
    roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES);
  can_errors.read_pdo_driver_state_error =
    roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);

  DriverCANErrors front_driver_can_errors;
  DriverCANErrors rear_driver_can_errors;

  front_driver_can_errors.motor_states_data_timed_out =
    front_driver_data.IsMotorStatesDataTimedOut();
  front_driver_can_errors.driver_state_data_timed_out =
    front_driver_data.IsDriverStateDataTimedOut();
  front_driver_can_errors.can_error = front_driver_data.IsCANError();
  front_driver_can_errors.heartbeat_timeout = front_driver_data.IsHeartbeatTimeout();

  rear_driver_can_errors.motor_states_data_timed_out = rear_driver_data.IsMotorStatesDataTimedOut();
  rear_driver_can_errors.driver_state_data_timed_out = rear_driver_data.IsDriverStateDataTimedOut();
  rear_driver_can_errors.can_error = rear_driver_data.IsCANError();
  rear_driver_can_errors.heartbeat_timeout = rear_driver_data.IsHeartbeatTimeout();

  can_errors.driver_errors.emplace(DriverNames::FRONT, front_driver_can_errors);
  can_errors.driver_errors.emplace(DriverNames::REAR, rear_driver_can_errors);

  system_ros_interface_->UpdateMsgErrors(can_errors);
}

void PantherSystem::UpdateFlagErrors()
{
  const auto front_driver_data = robot_driver_->GetData(DriverNames::FRONT);
  const auto rear_driver_data = robot_driver_->GetData(DriverNames::REAR);
  if (front_driver_data.IsFlagError() || rear_driver_data.IsFlagError()) {
    if (front_driver_data.IsFlagError()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, steady_clock_, 1000,
        "Front driver in error state:\n"
          << front_driver_data.GetFlagErrorLog());
    }

    if (rear_driver_data.IsFlagError()) {
      RCLCPP_WARN_STREAM_THROTTLE(
        logger_, steady_clock_, 1000,
        "Rear driver in error state:\n"
          << rear_driver_data.GetFlagErrorLog());
    }

    roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);

    HandleRobotDriverWriteOperation([this] { robot_driver_->AttemptErrorFlagReset(); });
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
  }
}

void PantherSystem::UpdateDriverStateDataTimedOut()
{
  if (
    robot_driver_->GetData(DriverNames::FRONT).IsDriverStateDataTimedOut() ||
    robot_driver_->GetData(DriverNames::REAR).IsDriverStateDataTimedOut()) {
    RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 1000, "PDO driver state timeout.");
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);
  } else {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, false);
  }
}

void PantherSystem::DiagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"No error detected."};

  const auto front_driver_data = robot_driver_->GetData(DriverNames::FRONT);
  if (front_driver_data.IsError()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Error detected.";

    husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
      status, front_driver_data.GetErrorMap(), "Front driver error: ");
  }

  const auto rear_driver_data = robot_driver_->GetData(DriverNames::REAR);
  if (rear_driver_data.IsError()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Error detected.";

    husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
      status, rear_driver_data.GetErrorMap(), "Rear driver error: ");
  }

  if (roboteq_error_filter_->IsError()) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Error detected.";

    husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
      status, roboteq_error_filter_->GetErrorMap(), "", " error");
  }

  status.summary(level, message);
}

void PantherSystem::DiagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level{diagnostic_updater::DiagnosticStatusWrapper::OK};
  std::string message{"Panther system status monitoring."};

  const auto front_driver_state = robot_driver_->GetData(DriverNames::FRONT).GetDriverState();
  const auto rear_driver_state = robot_driver_->GetData(DriverNames::REAR).GetDriverState();

  auto driver_states_with_names = {
    std::make_pair(std::string("Front"), front_driver_state),
    std::make_pair(std::string("Rear"), rear_driver_state)};

  for (const auto & [driver_name, driver_state] : driver_states_with_names) {
    status.add(driver_name + " driver voltage (V)", driver_state.GetVoltage());
    status.add(driver_name + " driver current (A)", driver_state.GetCurrent());
    status.add(driver_name + " driver temperature (\u00B0C)", driver_state.GetTemperature());
    status.add(
      driver_name + " driver heatsink temperature (\u00B0C)",
      driver_state.GetHeatsinkTemperature());
  }
  status.summary(level, message);
}

std::vector<float> PantherSystem::GetSpeedCommands() const
{
  return {
    static_cast<float>(hw_commands_velocities_[0]), static_cast<float>(hw_commands_velocities_[1]),
    static_cast<float>(hw_commands_velocities_[2]), static_cast<float>(hw_commands_velocities_[3])};
}

}  // namespace husarion_ugv_hardware_interfaces

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  husarion_ugv_hardware_interfaces::PantherSystem, hardware_interface::SystemInterface)
