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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DATA_CONVERTERS_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DATA_CONVERTERS_HPP_

#include <algorithm>
#include <bitset>
#include <cstdint>
#include <string>
#include <vector>

#include "husarion_ugv_msgs/msg/fault_flag.hpp"
#include "husarion_ugv_msgs/msg/runtime_error.hpp"
#include "husarion_ugv_msgs/msg/script_flag.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"
#include "husarion_ugv_hardware_interfaces/utils.hpp"

namespace husarion_ugv_hardware_interfaces
{

struct DrivetrainSettings
{
  float motor_torque_constant;
  float gear_ratio;
  float gearbox_efficiency;
  float encoder_resolution;
  float max_rpm_motor_speed;
};

/**
 * @brief Class for converting velocity commands (between SI units and raw Roboteq cmd)
 */
class RoboteqVelocityCommandConverter
{
public:
  RoboteqVelocityCommandConverter(const DrivetrainSettings & drivetrain_settings);
  std::int32_t Convert(const float cmd) const
  {
    return LimitCmd(cmd * radians_per_second_to_roboteq_cmd_);
  }

private:
  inline std::int32_t LimitCmd(const std::int32_t cmd) const
  {
    return std::clamp(cmd, -kMaxRoboteqCmdValue, kMaxRoboteqCmdValue);
  }

  float radians_per_second_to_roboteq_cmd_;
  static constexpr std::int32_t kMaxRoboteqCmdValue = 1000;
};

/**
 * @brief Class for storing and converting (between raw Roboteq data and SI units) motor
 * state (position, velocity, torque)
 */
class MotorState
{
public:
  MotorState(const DrivetrainSettings & drivetrain_settings);

  void SetData(const MotorDriverState & motor_state) { motor_state_ = motor_state; };

  float GetPosition() const { return motor_state_.pos * roboteq_pos_feedback_to_radians_; }
  float GetVelocity() const
  {
    return motor_state_.vel * roboteq_vel_feedback_to_radians_per_second_;
  }
  float GetTorque() const
  {
    return motor_state_.current * roboteq_current_feedback_to_newton_meters_;
  }

private:
  float roboteq_pos_feedback_to_radians_;
  float roboteq_vel_feedback_to_radians_per_second_;
  float roboteq_current_feedback_to_newton_meters_;

  MotorDriverState motor_state_ = {0, 0, 0, {0, 0}, {0, 0}};
};

/**
 * @brief Base class for different errors that are based on setting bits of one byte
 */
class FlagError
{
public:
  /**
   * @brief Sets which flags should be ignored when checking if an error occurred when converting
   * to message true data still will be set
   */
  FlagError(
    const std::vector<std::string> & flag_names,
    const std::vector<std::string> & suppressed_flags_names = {});

  virtual ~FlagError() = default;

  void SetData(const std::uint8_t flags) { flags_ = flags; }

  bool IsError() const { return (flags_ & (~suppressed_flags_)).any(); }

  std::string GetErrorLog() const;

protected:
  const std::vector<std::string> flag_names_;

  std::bitset<8> suppressed_flags_ = 0;
  std::bitset<8> flags_ = 0;
};

class FaultFlag : public FlagError
{
public:
  FaultFlag();
  husarion_ugv_msgs::msg::FaultFlag GetMessage() const;
  std::map<std::string, bool> GetErrorMap() const;
};

class ScriptFlag : public FlagError
{
public:
  ScriptFlag();
  husarion_ugv_msgs::msg::ScriptFlag GetMessage() const;
  std::map<std::string, bool> GetErrorMap() const;
};

class RuntimeError : public FlagError
{
public:
  RuntimeError();
  husarion_ugv_msgs::msg::RuntimeError GetMessage() const;
  std::map<std::string, bool> GetErrorMap() const;
};

/**
 * @brief Class for storing and converting the current state of the Roboteq drivers (temperature,
 * voltage and battery current)
 */
class RoboteqDriverState
{
public:
  RoboteqDriverState() {}

  void SetTemperature(const std::int16_t temp) { temp_ = temp; };
  void SetHeatsinkTemperature(const std::int16_t heatsink_temp) { heatsink_temp_ = heatsink_temp; };
  void SetVoltage(const std::uint16_t voltage) { voltage_ = voltage; };
  void SetBatteryCurrent1(const std::int16_t battery_current_1)
  {
    battery_current_1_ = battery_current_1;
  };
  void SetBatteryCurrent2(const std::int16_t battery_current_2)
  {
    battery_current_2_ = battery_current_2;
  };

  float GetTemperature() const { return temp_; }
  float GetHeatsinkTemperature() const { return heatsink_temp_; }
  float GetVoltage() const { return voltage_ / 10.0; }
  float GetCurrent() const { return (battery_current_1_ + battery_current_2_) / 10.0; }

private:
  std::int16_t temp_ = 0;
  std::int16_t heatsink_temp_ = 0;
  std::uint16_t voltage_ = 0;
  std::int16_t battery_current_1_ = 0;
  std::int16_t battery_current_2_ = 0;
};

/**
 * @brief Class that combines all the data that the one Roboteq driver provides
 */
class DriverData
{
public:
  DriverData(const DrivetrainSettings & drivetrain_settings)
  : channel_1_motor_state_(drivetrain_settings), channel_2_motor_state_(drivetrain_settings)
  {
  }

  void SetMotorsStates(
    const MotorDriverState & channel_1_state, const MotorDriverState & channel_2_state,
    const bool data_timed_out);
  void SetDriverState(const DriverState & state, const bool data_timed_out);
  void SetCANError(const bool can_error) { can_error_ = can_error; }
  void SetHeartbeatTimeout(const bool heartbeat_timeout) { heartbeat_timeout_ = heartbeat_timeout; }

  bool IsFlagError() const
  {
    return fault_flags_.IsError() || script_flags_.IsError() ||
           channel_1_runtime_error_.IsError() || channel_2_runtime_error_.IsError();
  }

  bool IsError() const
  {
    return IsFlagError() || motor_states_data_timed_out_ || driver_state_data_timed_out_ ||
           can_error_ || heartbeat_timeout_;
  }

  /**
   * @brief Returns motor state data for the given channel
   *
   * @param channel 1 or 2
   * @return motor state data
   * @throws std::runtime_error if invalid channel number
   */
  const MotorState & GetMotorState(const std::uint8_t channel) const;

  const RoboteqDriverState & GetDriverState() const { return driver_state_; }

  bool IsMotorStatesDataTimedOut() const { return motor_states_data_timed_out_; }
  bool IsDriverStateDataTimedOut() const { return driver_state_data_timed_out_; }
  bool IsCANError() const { return can_error_; }
  bool IsHeartbeatTimeout() const { return heartbeat_timeout_; }

  const FaultFlag & GetFaultFlag() const { return fault_flags_; }
  const ScriptFlag & GetScriptFlag() const { return script_flags_; }

  /**
   * @brief Returns runtime error flags for the given channel
   *
   * @param channel 1 or 2
   * @return runtime error flags
   * @throws std::runtime_error if invalid channel number
   */
  const RuntimeError & GetRuntimeError(const std::uint8_t channel) const;

  std::string GetFlagErrorLog() const;

  std::map<std::string, bool> GetFlagErrorMap() const;
  std::map<std::string, bool> GetErrorMap() const;

private:
  MotorState channel_1_motor_state_;
  MotorState channel_2_motor_state_;

  RoboteqDriverState driver_state_;

  FaultFlag fault_flags_;
  ScriptFlag script_flags_;
  RuntimeError channel_1_runtime_error_;
  RuntimeError channel_2_runtime_error_;

  bool motor_states_data_timed_out_ = false;
  bool driver_state_data_timed_out_ = false;
  bool can_error_ = false;
  bool heartbeat_timeout_ = false;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_DATA_CONVERTERS_HPP_
