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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <system_error>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/utils.hpp"

namespace husarion_ugv_hardware_interfaces
{

// All ids and sub ids were read directly from the eds file. Lely CANopen doesn't have the option
// to parse them based on the ParameterName. Additionally between version v60 and v80
// ParameterName changed, for example: Cmd_ESTOP (old), Cmd_ESTOP Emergency Shutdown (new) As
// parameter names changed, but ids stayed the same, it will be better to just use ids directly
struct RoboteqCANObjects
{
  static constexpr std::uint16_t cmd_id = 0x2000;
  static constexpr std::uint16_t position_id = 0x2104;
  static constexpr std::uint16_t velocity_id = 0x2107;
  static constexpr std::uint16_t current_id = 0x2100;

  static constexpr CANopenObject flags = {0x2106, 7};

  static constexpr CANopenObject mcu_temp = {0x210F, 1};
  static constexpr CANopenObject heatsink_temp = {0x210F, 2};
  static constexpr CANopenObject battery_voltage = {0x210D, 2};
  static constexpr CANopenObject battery_current_1 = {0x210C, 1};
  static constexpr CANopenObject battery_current_2 = {0x210C, 2};

  static constexpr CANopenObject reset_script = {0x2018, 0};
  static constexpr CANopenObject turn_on_e_stop = {0x200C, 0};       // Cmd_ESTOP
  static constexpr CANopenObject turn_off_e_stop = {0x200D, 0};      // Cmd_MGO
  static constexpr CANopenObject turn_on_safety_stop = {0x202C, 0};  // Cmd_SFT
};

MotorDriverState RoboteqMotorDriver::ReadState()
{
  MotorDriverState state;

  if (auto driver = driver_.lock()) {
    state.pos = driver->rpdo_mapped[RoboteqCANObjects::position_id][channel_];
    state.vel = driver->rpdo_mapped[RoboteqCANObjects::velocity_id][channel_];
    state.current = driver->rpdo_mapped[RoboteqCANObjects::current_id][channel_];
    state.pos_timestamp = driver->GetPositionTimestamp(channel_);
    state.vel_current_timestamp = driver->GetSpeedCurrentTimestamp(channel_);
  }

  return state;
}

void RoboteqMotorDriver::SendCmdVel(const std::int32_t cmd)
{
  if (auto driver = driver_.lock()) {
    driver->tpdo_mapped[RoboteqCANObjects::cmd_id][channel_] = cmd;
    driver->tpdo_mapped[RoboteqCANObjects::cmd_id][channel_].WriteEvent();
  }
}

void RoboteqMotorDriver::TurnOnSafetyStop()
{
  try {
    if (auto driver = driver_.lock()) {
      driver->SyncSDOWrite<std::uint8_t>(
        RoboteqCANObjects::turn_on_safety_stop.id, RoboteqCANObjects::turn_on_safety_stop.subid,
        channel_);
    }
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to turn on safety stop on channel " + std::to_string(channel_) + ": " +
      std::string(e.what()));
  }
}

RoboteqDriver::RoboteqDriver(
  const std::shared_ptr<lely::canopen::AsyncMaster> & async_master, const std::uint8_t id,
  const std::chrono::milliseconds & sdo_operation_timeout_ms)
: lely::canopen::LoopDriver(*async_master, id), sdo_operation_timeout_ms_(sdo_operation_timeout_ms)
{
}

std::future<void> RoboteqDriver::Boot()
{
  std::lock_guard<std::mutex> lck(boot_mtx_);
  boot_promise_ = std::promise<void>();
  std::future<void> future = boot_promise_.get_future();

  if (!LoopDriver::Boot()) {
    throw std::runtime_error("Boot failed.");
  }
  return future;
}

DriverState RoboteqDriver::ReadState()
{
  DriverState state;

  std::int32_t flags = static_cast<std::int32_t>(
    rpdo_mapped[RoboteqCANObjects::flags.id][RoboteqCANObjects::flags.subid]);
  state.fault_flags = GetByte(flags, 0);
  state.runtime_stat_flag_channel_1 = GetByte(flags, 1);
  state.runtime_stat_flag_channel_2 = GetByte(flags, 2);
  state.script_flags = GetByte(flags, 3);

  state.mcu_temp = rpdo_mapped[RoboteqCANObjects::mcu_temp.id][RoboteqCANObjects::mcu_temp.subid];
  state.heatsink_temp =
    rpdo_mapped[RoboteqCANObjects::heatsink_temp.id][RoboteqCANObjects::heatsink_temp.subid];
  state.battery_voltage =
    rpdo_mapped[RoboteqCANObjects::battery_voltage.id][RoboteqCANObjects::battery_voltage.subid];
  state.battery_current_1 = rpdo_mapped[RoboteqCANObjects::battery_current_1.id]
                                       [RoboteqCANObjects::battery_current_1.subid];
  state.battery_current_2 = rpdo_mapped[RoboteqCANObjects::battery_current_2.id]
                                       [RoboteqCANObjects::battery_current_2.subid];

  {
    std::lock_guard<std::mutex> lck_fa(flags_current_timestamp_mtx_);
    state.flags_current_timestamp = flags_current_timestamp_;
  }

  {
    std::lock_guard<std::mutex> lck_vt(voltages_temps_timestamp_mtx_);
    state.voltages_temps_timestamp = last_voltages_temps_timestamp_;
  }

  return state;
}

void RoboteqDriver::ResetScript()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::reset_script.id, RoboteqCANObjects::reset_script.subid, 2);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to reset Roboteq script: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOnEStop()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_on_e_stop.id, RoboteqCANObjects::turn_on_e_stop.subid, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn on E-stop: " + std::string(e.what()));
  }
}

void RoboteqDriver::TurnOffEStop()
{
  try {
    SyncSDOWrite<std::uint8_t>(
      RoboteqCANObjects::turn_off_e_stop.id, RoboteqCANObjects::turn_off_e_stop.subid, 1);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Error when trying to turn off E-stop: " + std::string(e.what()));
  }
}

void RoboteqDriver::AddMotorDriver(
  const MotorNames name, std::shared_ptr<MotorDriverInterface> motor_driver)
{
  if (std::dynamic_pointer_cast<RoboteqMotorDriver>(motor_driver) == nullptr) {
    throw std::runtime_error("Motor driver is not of type RoboteqMotorDriver");
  }
  motor_drivers_.emplace(name, motor_driver);
}

std::shared_ptr<MotorDriverInterface> RoboteqDriver::GetMotorDriver(const MotorNames name)
{
  auto it = motor_drivers_.find(name);
  if (it == motor_drivers_.end()) {
    throw std::runtime_error(
      "Motor driver with name '" + MotorNamesToString(name) + "' does not exist");
  }

  return it->second;
}

template <typename T>
void RoboteqDriver::SyncSDOWrite(
  const std::uint16_t index, const std::uint8_t subindex, const T data)
{
  std::mutex mtx;
  std::condition_variable cv;
  std::error_code err_code;

  try {
    SubmitWrite(
      index, subindex, data,
      [&mtx, &cv, &err_code](
        std::uint8_t, std::uint16_t, std::uint8_t, std::error_code ec) mutable {
        {
          std::lock_guard<std::mutex> lck_g(mtx);
          if (ec) {
            err_code = ec;
          }
        }
        cv.notify_one();
      },
      sdo_operation_timeout_ms_);
  } catch (const lely::canopen::SdoError & e) {
    throw std::runtime_error("SDO write error, message: " + std::string(e.what()));
  }

  std::unique_lock<std::mutex> lck(mtx);
  cv.wait(lck);

  if (err_code) {
    throw std::runtime_error("Error msg: " + err_code.message());
  }
}

void RoboteqDriver::OnBoot(
  const lely::canopen::NmtState st, const char es, const std::string & what) noexcept
{
  LoopDriver::OnBoot(st, es, what);
  std::lock_guard<std::mutex> lck(boot_mtx_);

  try {
    if (!es || es == 'L') {
      boot_promise_.set_value();
    } else {
      boot_promise_.set_exception(std::make_exception_ptr(std::runtime_error(what)));
    }
  } catch (const std::future_error & e) {
    if (e.code() == std::make_error_code(std::future_errc::promise_already_satisfied)) {
      std::cerr << "An exception occurred while setting boot promise: " << e.what() << std::endl;
    }
  }
}

void RoboteqDriver::OnRpdoWrite(const std::uint16_t idx, const std::uint8_t subidx) noexcept
{
  if (idx == RoboteqCANObjects::position_id) {
    if (subidx != kChannel1 && subidx != kChannel2) {
      return;
    }
    std::lock_guard<std::mutex> lck(position_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_position_timestamps_.at(subidx));
  } else if (idx == RoboteqCANObjects::velocity_id) {
    if (subidx != kChannel1 && subidx != kChannel2) {
      return;
    }
    std::lock_guard<std::mutex> lck(speed_current_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_speed_current_timestamps_.at(subidx));
  } else if (idx == RoboteqCANObjects::flags.id && subidx == RoboteqCANObjects::flags.subid) {
    std::lock_guard<std::mutex> lck(flags_current_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &flags_current_timestamp_);
  } else if (
    idx == RoboteqCANObjects::battery_voltage.id &&
    subidx == RoboteqCANObjects::battery_voltage.subid) {
    std::lock_guard<std::mutex> lck(voltages_temps_timestamp_mtx_);
    clock_gettime(CLOCK_MONOTONIC, &last_voltages_temps_timestamp_);
  }
}

}  // namespace husarion_ugv_hardware_interfaces
