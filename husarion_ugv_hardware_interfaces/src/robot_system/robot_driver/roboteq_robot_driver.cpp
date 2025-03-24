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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"

#include <chrono>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "lely/util/chrono.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_robot_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

RoboteqRobotDriver::RoboteqRobotDriver(
  const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings,
  const std::chrono::milliseconds activate_wait_time)
: canopen_settings_(canopen_settings),
  drivetrain_settings_(drivetrain_settings),
  canopen_manager_(canopen_settings),
  roboteq_vel_cmd_converter_(drivetrain_settings),
  pdo_motor_states_timeout_ms_(canopen_settings.pdo_motor_states_timeout_ms),
  pdo_driver_state_timeout_ms_(canopen_settings.pdo_driver_state_timeout_ms),
  activate_wait_time_(activate_wait_time)
{
}

void RoboteqRobotDriver::Initialize()
{
  if (initialized_) {
    return;
  }

  try {
    canopen_manager_.Initialize();
    DefineDrivers();
    for (auto & [name, driver] : drivers_) {
      data_.emplace(name, DriverData(drivetrain_settings_));
    }

    canopen_manager_.Activate();
    BootDrivers();

  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to initialize robot driver: " + std::string(e.what()));
  }

  initialized_ = true;
}

void RoboteqRobotDriver::Deinitialize()
{
  drivers_.clear();
  data_.clear();
  canopen_manager_.Deinitialize();
  initialized_ = false;
}

void RoboteqRobotDriver::Activate()
{
  // Activation procedure - it is necessary to first reset scripts, wait for a bit (1 second)
  // and then send 0 commands for some time (also 1 second)

  for (auto & [name, driver] : drivers_) {
    try {
      driver->ResetScript();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Reset Roboteq script exception on " + DriverNamesToString(name) +
        " driver : " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(activate_wait_time_);

  for (auto & [name, driver] : drivers_) {
    try {
      driver->GetMotorDriver(MotorNames::LEFT)->SendCmdVel(0);
      driver->GetMotorDriver(MotorNames::RIGHT)->SendCmdVel(0);
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Send 0 command exception on " + DriverNamesToString(name) +
        "driver : " + std::string(e.what()));
    }
  }

  std::this_thread::sleep_for(activate_wait_time_);
}

void RoboteqRobotDriver::UpdateCommunicationState()
{
  for (auto & [name, driver] : drivers_) {
    data_.at(name).SetCANError(driver->IsCANError());
    data_.at(name).SetHeartbeatTimeout(driver->IsHeartbeatTimeout());
  }
}

void RoboteqRobotDriver::UpdateMotorsState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : drivers_) {
    const auto left_state = driver->GetMotorDriver(MotorNames::LEFT)->ReadState();
    const auto right_state = driver->GetMotorDriver(MotorNames::RIGHT)->ReadState();

    SetMotorsStates(data_.at(name), left_state, right_state, current_time);
  }

  UpdateCommunicationState();

  if (std::any_of(
        data_.begin(), data_.end(), [](const auto & data) { return data.second.IsCANError(); })) {
    throw std::runtime_error("CAN error.");
  }

  if (std::any_of(data_.begin(), data_.end(), [](const auto & data) {
        return data.second.IsHeartbeatTimeout();
      })) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

void RoboteqRobotDriver::UpdateDriversState()
{
  timespec current_time;
  clock_gettime(CLOCK_MONOTONIC, &current_time);

  for (auto & [name, driver] : drivers_) {
    SetDriverState(data_.at(name), driver->ReadState(), current_time);
  }

  UpdateCommunicationState();

  if (std::any_of(
        data_.begin(), data_.end(), [](const auto & data) { return data.second.IsCANError(); })) {
    throw std::runtime_error("CAN error.");
  }

  if (std::any_of(data_.begin(), data_.end(), [](const auto & data) {
        return data.second.IsHeartbeatTimeout();
      })) {
    throw std::runtime_error("Motor controller heartbeat timeout.");
  }
}

const DriverData & RoboteqRobotDriver::GetData(const DriverNames name)
{
  if (data_.find(name) == data_.end()) {
    throw std::runtime_error("Data with name '" + DriverNamesToString(name) + "' does not exist.");
  }

  return data_.at(name);
}

void RoboteqRobotDriver::TurnOnEStop()
{
  for (auto & [name, driver] : drivers_) {
    try {
      driver->TurnOnEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to turn on E-stop on " + DriverNamesToString(name) +
        " driver: " + std::string(e.what()));
    }
  }
}

void RoboteqRobotDriver::TurnOffEStop()
{
  for (auto & [name, driver] : drivers_) {
    try {
      driver->TurnOffEStop();
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to turn off E-stop on " + DriverNamesToString(name) +
        " driver: " + std::string(e.what()));
    }
  }
}

bool RoboteqRobotDriver::CommunicationError()
{
  return std::any_of(data_.begin(), data_.end(), [](const auto & data) {
    return data.second.IsCANError() || data.second.IsHeartbeatTimeout();
  });
}

void RoboteqRobotDriver::SetMotorsStates(
  DriverData & data, const MotorDriverState & left_state, const MotorDriverState & right_state,
  const timespec & current_time)
{
  const bool data_timed_out =
    DataTimeout(current_time, left_state.pos_timestamp, pdo_motor_states_timeout_ms_) ||
    DataTimeout(current_time, left_state.vel_current_timestamp, pdo_motor_states_timeout_ms_) ||
    DataTimeout(current_time, right_state.pos_timestamp, pdo_motor_states_timeout_ms_) ||
    DataTimeout(current_time, right_state.vel_current_timestamp, pdo_motor_states_timeout_ms_);

  // Channel 1 - right, Channel 2 - left
  data.SetMotorsStates(right_state, left_state, data_timed_out);
}

void RoboteqRobotDriver::SetDriverState(
  DriverData & data, const DriverState & state, const timespec & current_time)
{
  const bool data_timed_out =
    DataTimeout(current_time, state.flags_current_timestamp, pdo_driver_state_timeout_ms_) ||
    DataTimeout(current_time, state.voltages_temps_timestamp, pdo_driver_state_timeout_ms_);

  data.SetDriverState(state, data_timed_out);
}

bool RoboteqRobotDriver::DataTimeout(
  const timespec & current_time, const timespec & data_timestamp,
  const std::chrono::milliseconds & timeout)
{
  return lely::util::from_timespec(current_time) - lely::util::from_timespec(data_timestamp) >
         timeout;
}

void RoboteqRobotDriver::BootDrivers()
{
  for (auto & [name, driver] : drivers_) {
    const auto name_str = DriverNamesToString(name);
    try {
      auto driver_future = driver->Boot();
      auto driver_status = driver_future.wait_for(std::chrono::seconds(5));

      if (driver_status == std::future_status::ready) {
        try {
          driver_future.get();
        } catch (const std::exception & e) {
          throw std::runtime_error(
            "Boot for " + name_str + " driver failed with exception: " + std::string(e.what()));
        }
      } else {
        throw std::runtime_error("Boot for " + name_str + " driver timed out or failed.");
      }

    } catch (const std::exception & e) {
      throw std::runtime_error(
        "An exception occurred while trying to Boot " + name_str + " driver " +
        std::string(e.what()));
    }
  }
}

}  // namespace husarion_ugv_hardware_interfaces
