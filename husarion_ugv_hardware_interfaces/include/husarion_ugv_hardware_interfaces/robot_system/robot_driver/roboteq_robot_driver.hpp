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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_ROBOT_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_ROBOT_DRIVER_HPP_

#include <chrono>
#include <string>
#include <vector>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

struct MotorChannels
{
  static constexpr std::uint8_t LEFT = RoboteqDriver::kChannel2;
  static constexpr std::uint8_t RIGHT = RoboteqDriver::kChannel1;
};

/**
 * @brief Abstract class implementing RobotDriver for robots using Roboteq drivers.
 * It uses canopen_manager for communication with Roboteq controllers.
 * This class implements the activation procedure for controllers, which involves resetting the
 * script and sending an initial 0 command. It also provides methods to get data feedback and send
 * commands. Data is converted between raw Roboteq formats and SI units using
 * roboteq_data_converters.
 */
class RoboteqRobotDriver : public RobotDriverInterface
{
public:
  RoboteqRobotDriver(
    const CANopenSettings & canopen_settings, const DrivetrainSettings & drivetrain_settings,
    const std::chrono::milliseconds activate_wait_time = std::chrono::milliseconds(1000));

  ~RoboteqRobotDriver()
  {
    drivers_.clear();
    Deinitialize();
  };

  /**
   * @brief Starts CAN communication and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize() override;

  /**
   * @brief Deinitialize can communication
   */
  void Deinitialize() override;

  /**
   * @brief Activate procedure for Roboteq drivers - reset scripts and send 0 commands on both
   * channels. Blocking function, takes around 2 seconds to finish
   *
   * @exception std::runtime_error if any procedure step fails
   */
  void Activate() override;

  /**
   * @brief Updates current communication state with Roboteq drivers
   *
   * @exception std::runtime_error if CAN error was detected
   */
  void UpdateCommunicationState() override;

  /**
   * @brief Updates current motors' state (position, velocity, current).
   *
   * @exception std::runtime_error if CAN error was detected
   */
  void UpdateMotorsState() override;

  /**
   * @brief Updates current Roboteq driver state (flags, temperatures, voltage, battery current)
   *
   * @exception std::runtime_error if CAN error was detected
   */
  void UpdateDriversState() override;

  /**
   * @brief Turns on Roboteq E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOnEStop() override;

  /**
   * @brief Turns off Roboteq E-stop
   *
   * @exception std::runtime_error if any operation returns error
   */
  void TurnOffEStop() override;

  /**
   * @brief Get data feedback from the driver
   *
   * @param name name of the data to get
   *
   * @return data feedback
   * @exception std::runtime_error if data with the given name does not exist
   */
  const DriverData & GetData(const DriverNames name) override;

  bool CommunicationError() override;

protected:
  /**
   * @brief This method defines driver objects and adds motor drivers for them.
   */
  virtual void DefineDrivers() = 0;

  RoboteqVelocityCommandConverter & GetCmdVelConverter() { return roboteq_vel_cmd_converter_; }
  CANopenSettings & GetCANopenSettings() { return canopen_settings_; }
  CANopenManager & GetCANopenManager() { return canopen_manager_; }

  CANopenSettings canopen_settings_;
  DrivetrainSettings drivetrain_settings_;

  CANopenManager canopen_manager_;
  std::unordered_map<DriverNames, std::shared_ptr<DriverInterface>> drivers_;

private:
  void SetMotorsStates(
    DriverData & data, const MotorDriverState & left_state, const MotorDriverState & right_state,
    const timespec & current_time);
  void SetDriverState(DriverData & data, const DriverState & state, const timespec & current_time);
  bool DataTimeout(
    const timespec & current_time, const timespec & data_timestamp,
    const std::chrono::milliseconds & timeout);
  void BootDrivers();

  bool initialized_ = false;

  std::unordered_map<DriverNames, DriverData> data_;
  RoboteqVelocityCommandConverter roboteq_vel_cmd_converter_;

  const std::chrono::milliseconds pdo_motor_states_timeout_ms_;
  const std::chrono::milliseconds pdo_driver_state_timeout_ms_;
  const std::chrono::milliseconds activate_wait_time_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_ROBOTEQ_ROBOT_DRIVER_HPP_
