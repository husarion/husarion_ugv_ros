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
#include <condition_variable>
#include <cstdint>
#include <exception>
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
    // Timestamps first: the writer stores position and velocity before their timestamps, so those
    // values are at least as new as the timestamps. The current has no timestamp of its own and
    // can be one frame older or newer.
    state.pos_timestamp = driver->GetPositionTimestamp(channel_);
    state.vel_current_timestamp = driver->GetSpeedCurrentTimestamp(channel_);
    state.pos = driver->GetPosition(channel_);
    state.vel = driver->GetVelocity(channel_);
    state.current = driver->GetCurrent(channel_);
  }

  return state;
}

void RoboteqMotorDriver::SendCmdVel(const std::int32_t cmd)
{
  if (auto driver = driver_.lock()) {
    driver->PostCmdVel(channel_, cmd);
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

std::weak_ptr<RoboteqDriver> RoboteqDriver::GetWeakSelf()
{
  auto weak_self = weak_from_this();
  if (weak_self.expired()) {
    throw std::runtime_error("RoboteqDriver must be owned by a std::shared_ptr.");
  }
  return weak_self;
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

  // Timestamps first, as in RoboteqMotorDriver::ReadState(): the flags and the battery voltage
  // are stored before their timestamps, so they are at least as new as them.
  state.flags_current_timestamp =
    ToTimespec(flags_current_timestamp_ns_.load(std::memory_order_acquire));
  state.voltages_temps_timestamp =
    ToTimespec(last_voltages_temps_timestamp_ns_.load(std::memory_order_acquire));

  const std::int32_t flags = last_flags_.load(std::memory_order_acquire);
  state.fault_flags = GetByte(flags, 0);
  state.runtime_stat_flag_channel_1 = GetByte(flags, 1);
  state.runtime_stat_flag_channel_2 = GetByte(flags, 2);
  state.script_flags = GetByte(flags, 3);

  state.mcu_temp = last_mcu_temp_.load(std::memory_order_acquire);
  state.heatsink_temp = last_heatsink_temp_.load(std::memory_order_acquire);
  state.battery_voltage = last_battery_voltage_.load(std::memory_order_acquire);
  state.battery_current_1 = last_battery_current_1_.load(std::memory_order_acquire);
  state.battery_current_2 = last_battery_current_2_.load(std::memory_order_acquire);

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
      RoboteqCANObjects::turn_on_e_stop.id, RoboteqCANObjects::turn_on_e_stop.subid, 1,
      SdoLateWrite::kKeepQueued);
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
  const std::uint16_t index, const std::uint8_t subindex, const T data,
  const SdoLateWrite late_write)
{
  // Shared with the posted task and the confirmation, which can outlive this call.
  struct WaitState
  {
    std::mutex mtx;
    std::condition_variable cv;
    std::error_code err_code;
    std::string err_msg;
    bool submitted = false;
    bool done = false;
    bool abandoned = false;
  };
  auto state = std::make_shared<WaitState>();

  // The write runs on the master's executor, so a caller that gives up before it ran can still
  // drop it, depending on late_write. The task holds a weak pointer, so a task left in the queue
  // never reaches a destroyed driver. Never call this from the master's executor or a driver loop,
  // it would wait on its own queue.
  master.GetExecutor().post([weak_self = GetWeakSelf(), state, index, subindex, data]() {
    // The master co-owns the driver while this runs. The drivers must only be dropped with the
    // master loop stopped, or ~LoopDriver could run on the master's own executor.
    auto self = weak_self.lock();
    if (!self) {
      return;
    }
    {
      std::lock_guard<std::mutex> lck_g(state->mtx);
      if (state->abandoned) {
        return;
      }
      state->submitted = true;
    }

    try {
      self->SubmitWrite(
        index, subindex, data,
        [state](std::uint8_t, std::uint16_t, std::uint8_t, std::error_code ec) {
          {
            std::lock_guard<std::mutex> lck_g(state->mtx);
            if (ec) {
              state->err_code = ec;
            }
            state->done = true;
          }
          state->cv.notify_one();
        },
        self->sdo_operation_timeout_ms_);
    } catch (const lely::canopen::SdoError & e) {
      {
        std::lock_guard<std::mutex> lck_g(state->mtx);
        state->err_code = e.code();
        state->err_msg = e.what();
        state->done = true;
      }
      state->cv.notify_one();
    } catch (const std::exception & e) {
      {
        std::lock_guard<std::mutex> lck_g(state->mtx);
        state->err_code = std::make_error_code(std::errc::io_error);
        state->err_msg = e.what();
        state->done = true;
      }
      state->cv.notify_one();
    }
  });

  // The predicate covers a confirmation that arrives before this thread starts waiting.
  std::unique_lock<std::mutex> lck(state->mtx);
  if (!state->cv.wait_for(
        lck, sdo_operation_timeout_ms_ + kSdoConfirmationTimeoutMargin,
        [&state]() { return state->done; })) {
    if (state->submitted) {
      throw std::runtime_error(
        "SDO write submitted but not confirmed in time, it may have reached the controller.");
    }
    if (late_write == SdoLateWrite::kKeepQueued) {
      throw std::runtime_error(
        "SDO write not sent in time: the CANopen master did not run it yet, it may still be "
        "sent.");
    }
    state->abandoned = true;
    throw std::runtime_error("SDO write not sent: the CANopen master did not run it in time.");
  }

  if (state->err_code) {
    throw std::runtime_error(
      "SDO write error, message: " +
      (state->err_msg.empty() ? state->err_code.message() : state->err_msg));
  }
}

void RoboteqDriver::PostCmdVel(const std::uint8_t channel, const std::int32_t cmd)
{
  const std::size_t slot = channel - kChannel1;
  pending_cmd_.at(slot).store(cmd, std::memory_order_release);
  const auto now = std::chrono::steady_clock::now().time_since_epoch().count();

  if (cmd_task_queued_.at(slot).exchange(true, std::memory_order_acq_rel)) {
    // The task of an earlier cycle has not run: a master that does not run it writes nothing.
    const auto queued_for = std::chrono::steady_clock::duration(
      now - cmd_task_posted_.at(slot).load(std::memory_order_relaxed));
    if (queued_for > kCmdTaskDeadline) {
      throw std::runtime_error(
        "Velocity command for channel " + std::to_string(channel) + " not written for " +
        std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(queued_for).count()) +
        " ms: the CANopen master does not run it.");
    }
  } else {
    cmd_task_posted_.at(slot).store(now, std::memory_order_relaxed);
    try {
      PostCmdVelTask(channel, slot);
    } catch (const std::exception & e) {
      // Otherwise every later command on this channel would wait for a task that does not exist.
      cmd_task_queued_.at(slot).store(false, std::memory_order_release);
      throw std::runtime_error("Posting the velocity command failed: " + std::string(e.what()));
    }
  }

  cmd_write_status_.ThrowIfFailed(slot);
}

void RoboteqDriver::PostCmdVelTask(const std::uint8_t channel, const std::size_t slot)
{
  master.GetExecutor().post([weak_self = GetWeakSelf(), channel, slot]() {
    auto self = weak_self.lock();
    if (!self) {
      return;
    }
    // Both sides use read-modify-write operations on the flag, so a command stored before a
    // PostCmdVel that found the flag set is visible here.
    self->cmd_task_queued_.at(slot).exchange(false, std::memory_order_acq_rel);
    const std::int32_t latest = self->pending_cmd_.at(slot).load(std::memory_order_acquire);
    // lely runs tasks in a noexcept callback, so an exception must not leave this task.
    try {
      self->tpdo_mapped[RoboteqCANObjects::cmd_id][channel] = latest;
      self->tpdo_mapped[RoboteqCANObjects::cmd_id][channel].WriteEvent();
      self->cmd_write_status_.RecordSuccess(slot);
    } catch (const std::exception & e) {
      self->cmd_write_status_.RecordFailure(
        slot, "channel " + std::to_string(channel) + ": " + e.what());
    }
  });
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
  timespec current_timestamp;
  clock_gettime(CLOCK_MONOTONIC, &current_timestamp);
  const std::int64_t now_ns = ToNanoseconds(current_timestamp);

  // This thread is the only writer. A failed read leaves the timestamp old, so it shows up as a
  // PDO timeout, and the first one is logged.
  try {
    if (idx == RoboteqCANObjects::position_id) {
      if (subidx != kChannel1 && subidx != kChannel2) {
        return;
      }
      last_positions_.at(subidx - kChannel1)
        .store(static_cast<std::int32_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
      last_position_timestamps_ns_.at(subidx - kChannel1).store(now_ns, std::memory_order_release);
    } else if (idx == RoboteqCANObjects::velocity_id) {
      if (subidx != kChannel1 && subidx != kChannel2) {
        return;
      }
      last_velocities_.at(subidx - kChannel1)
        .store(static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
      last_speed_current_timestamps_ns_.at(subidx - kChannel1)
        .store(now_ns, std::memory_order_release);
    } else if (idx == RoboteqCANObjects::current_id) {
      if (subidx != kChannel1 && subidx != kChannel2) {
        return;
      }
      last_currents_.at(subidx - kChannel1)
        .store(static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
    } else if (idx == RoboteqCANObjects::flags.id && subidx == RoboteqCANObjects::flags.subid) {
      last_flags_.store(
        static_cast<std::int32_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
      flags_current_timestamp_ns_.store(now_ns, std::memory_order_release);
    } else if (
      idx == RoboteqCANObjects::mcu_temp.id && subidx == RoboteqCANObjects::mcu_temp.subid) {
      last_mcu_temp_.store(
        static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
    } else if (
      idx == RoboteqCANObjects::heatsink_temp.id &&
      subidx == RoboteqCANObjects::heatsink_temp.subid) {
      last_heatsink_temp_.store(
        static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
    } else if (
      idx == RoboteqCANObjects::battery_current_1.id &&
      subidx == RoboteqCANObjects::battery_current_1.subid) {
      last_battery_current_1_.store(
        static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
    } else if (
      idx == RoboteqCANObjects::battery_current_2.id &&
      subidx == RoboteqCANObjects::battery_current_2.subid) {
      last_battery_current_2_.store(
        static_cast<std::int16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
    } else if (
      idx == RoboteqCANObjects::battery_voltage.id &&
      subidx == RoboteqCANObjects::battery_voltage.subid) {
      last_battery_voltage_.store(
        static_cast<std::uint16_t>(rpdo_mapped[idx][subidx]), std::memory_order_release);
      last_voltages_temps_timestamp_ns_.store(now_ns, std::memory_order_release);
    }
  } catch (const std::exception & e) {
    if (!rpdo_read_error_logged_.exchange(true)) {
      std::cerr << "Reading RPDO object 0x" << std::hex << idx << " sub " << std::dec
                << static_cast<int>(subidx)
                << " failed, further errors are not logged: " << e.what() << std::endl;
    }
  }
}

}  // namespace husarion_ugv_hardware_interfaces
