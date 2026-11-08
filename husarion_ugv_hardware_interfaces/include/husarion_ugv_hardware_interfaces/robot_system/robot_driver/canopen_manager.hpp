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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CANOPEN_MANAGER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CANOPEN_MANAGER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "lely/coapp/fiber_driver.hpp"
#include "lely/coapp/master.hpp"
#include "lely/ev/loop.hpp"
#include "lely/io2/linux/can.hpp"
#include "lely/io2/posix/poll.hpp"
#include "lely/io2/sys/io.hpp"
#include "lely/io2/sys/sigset.hpp"
#include "lely/io2/sys/timer.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"

namespace husarion_ugv_hardware_interfaces
{

struct CANopenObject
{
  const std::uint16_t id;
  const std::uint8_t subid;
};

struct CANopenSettings
{
  std::string can_interface_name;

  std::uint8_t master_can_id;
  std::unordered_map<DriverNames, std::uint8_t> driver_can_ids;

  std::chrono::milliseconds pdo_motor_states_timeout_ms;
  std::chrono::milliseconds pdo_driver_state_timeout_ms;
  std::chrono::milliseconds sdo_operation_timeout_ms;

  // Priority of the CANopen communication thread. It has to stay above the control loop (FIFO 50
  // by default), or the motor state timestamps go stale and the driver latches the e-stop.
  unsigned canopen_thread_sched_priority = 70;

  // Priority of the RPDO dispatch thread lely creates for every Roboteq driver, see
  // RoboteqDriver::OnRpdoWrite. 0 leaves the scheduling it inherited.
  unsigned rpdo_dispatch_sched_priority = 65;

  // CPUs the two threads above pin themselves to, -1 leaves them where the kernel puts them.
  int canopen_thread_cpu = -1;
  int rpdo_dispatch_cpu = -1;

  // Priority Initialize() sets on the calling thread. Every thread created from it afterwards
  // inherits it, among them the lely driver threads and the ROS interface executor. 0, the default,
  // leaves the calling thread as it is.
  unsigned configure_thread_sched_priority = 0;

  // Use the master dcf from config/limited_tpdo_rates, which writes TPDO inhibit and event timers
  // into the motor controllers on every slave boot.
  bool limit_tpdo_rates = false;
};

/**
 * @brief CANopenManager takes care of CANopen communication - creates master controller
 * and two Roboteq drivers (front and rear)
 */
class CANopenManager
{
public:
  CANopenManager(const CANopenSettings & canopen_settings);

  ~CANopenManager() { Deinitialize(); }

  /**
   * @brief Starts CANopen communication (in a new thread) and waits for boot to finish
   *
   * @exception std::runtime_error if boot fails
   */
  void Initialize();

  /**
   * @brief Stops CANopen communication - sends stop signal and waits
   */
  void Deinitialize();

  /**
   * @brief Stops the CANopen event loop and waits for its thread, but keeps the master, so the
   * drivers created on it can still be destroyed. Deinitialize() calls it too.
   */
  void StopCommunication();

  /**
   * @brief Activates CANopen communication thread. This method should be invoked after all objects
   * using this communication are created.
   *
   * @exception std::runtime_error if CAN communication not activated or not initialized
   */
  void Activate();

  /**
   * @brief Returns master controller
   *
   * @return std::shared_ptr<lely::canopen::AsyncMaster> master controller
   * @exception std::runtime_error if CANopenManager is not initialized
   */
  std::shared_ptr<lely::canopen::AsyncMaster> GetMaster()
  {
    if (!initialized_) {
      throw std::runtime_error("CANopenManager not initialized.");
    }
    return master_;
  }

private:
  void InitializeCANCommunication();

  /**
   * @brief Sets CAN communication started status and notifies other thread through the condition
   * variable
   *
   * @param result status of CAN communication started
   */
  void NotifyCANCommunicationStarted(const bool result);

  static constexpr std::chrono::seconds kCANopenCommunicationStartTimeout{10};

  bool initialized_ = false;

  std::atomic_bool canopen_communication_started_ = false;
  std::condition_variable canopen_communication_started_cond_;
  std::mutex canopen_communication_started_mtx_;

  std::thread canopen_communication_thread_;

  std::shared_ptr<lely::io::Context> ctx_;
  std::shared_ptr<lely::ev::Loop> loop_;
  std::shared_ptr<lely::io::Poll> poll_;
  std::shared_ptr<lely::ev::Executor> exec_;
  std::shared_ptr<lely::io::Timer> timer_;
  std::shared_ptr<lely::io::CanController> ctrl_;
  std::shared_ptr<lely::io::CanChannel> chan_;
  std::shared_ptr<lely::canopen::AsyncMaster> master_;

  const CANopenSettings canopen_settings_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CANOPEN_MANAGER_HPP_
