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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"

namespace husarion_ugv_hardware_interfaces
{

enum class EStopLatchSource : std::uint8_t {
  RELEASED = 0,
  // Latched since the driver started, the state the safety board powers up in.
  STARTUP,
  // Latched by this driver: a service call, a communication error or a shutdown.
  SOFTWARE,
  // The hardware line latched with no software trigger requested - the e-stop button, a Roboteq
  // fault or the safety board watchdog.
  HARDWARE,
};

/**
 * @brief E-stop state for the diagnostics, written on every e-stop state change.
 *
 * It lives apart from EStop and is shared with the diagnostic task, so a diagnostic tick that
 * races the hardware teardown never reads a destroyed object. The latch state is lock-free: the
 * diagnostic thread must never take e_stop_manipulation_mtx_, which the control loop needs on
 * every read. Only the refused-reset text has a mutex, shared by the reset service and the
 * diagnostic task alone.
 */
class EStopDiagnostics
{
public:
  struct Snapshot
  {
    EStopLatchSource source = EStopLatchSource::STARTUP;
    // Steady clock time of the last change, 0 when there was none since the driver started.
    std::int64_t last_change_ns = 0;
    std::string last_refused_reset;
    std::int64_t last_refused_reset_ns = 0;
  };

  void SetSource(const EStopLatchSource source);
  EStopLatchSource GetSource() const { return source_.load(); }
  void SetResetRefused(const std::string & reason);
  Snapshot Get() const;

  static std::int64_t SteadyNowNs();

private:
  std::atomic<EStopLatchSource> source_{EStopLatchSource::STARTUP};
  std::atomic<std::int64_t> last_change_ns_{0};

  mutable std::mutex refused_reset_mtx_;
  std::string last_refused_reset_;
  std::int64_t last_refused_reset_ns_ = 0;
};

/**
 * @class EStopInterface
 * @brief Abstract base class defining the interface for emergency stop detailed implementations.
 */
class EStopInterface
{
public:
  EStopInterface() {}

  virtual ~EStopInterface() = default;

  virtual bool ReadEStopState() = 0;
  virtual void TriggerEStop() = 0;
  virtual void ResetEStop() = 0;
  virtual std::shared_ptr<const EStopDiagnostics> GetDiagnostics() const = 0;
};

/**
 * @class EStop
 * @brief Implements the emergency stop interface.
 */
class EStop : public EStopInterface
{
public:
  EStop(
    std::shared_ptr<GPIOControllerInterface> gpio_controller,
    std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter,
    std::shared_ptr<RobotDriverInterface> robot_driver,
    std::shared_ptr<std::mutex> robot_driver_write_mtx, std::function<bool()> zero_velocity_check)
  : EStopInterface(),
    gpio_controller_(gpio_controller),
    roboteq_error_filter_(roboteq_error_filter),
    robot_driver_(robot_driver),
    robot_driver_write_mtx_(robot_driver_write_mtx),
    ZeroVelocityCheck(zero_velocity_check) {};

  virtual ~EStop() override = default;

  /**
   * @brief Checks the emergency stop state.
   *
   *   1. Check if ESTOP GPIO pin is not active. If is not it means that E-Stop is triggered by
   *      another device within the robot's system (e.g., Roboteq controller or Safety Board),
   *      disabling the software Watchdog is necessary to prevent an uncontrolled reset.
   *   2. If there is a need, disable software Watchdog using
   *      GPIOControllerInterface::EStopTrigger method.
   *   3. Return ESTOP GPIO pin state.
   */
  bool ReadEStopState() override;

  /**
   * @brief Triggers the emergency stop.
   *
   *   1. Interrupt the E-Stop resetting process if it is in progress.
   *   2. Attempt to trigger the E-Stop using GPIO by disabling the software-controlled watchdog.
   *   3. If successful, set e_stop_triggered_ to true; otherwise, throw a std::runtime_error
   *      exception.
   *
   * @throws std::runtime_error if triggering the E-stop using GPIO fails.
   */
  void TriggerEStop() override;

  /**
   * @brief Resets the emergency stop.
   *
   *   1. Verify that the last velocity commands are zero to prevent an uncontrolled robot movement
   *      after an E-stop state change.
   *   2. Attempt to reset the E-Stop using GPIO by manipulating the ESTOP GPIO pin. This operation
   *      may take some time and can be interrupted by the E-Stop trigger process.
   *   3. Set the clear_error flag to allow for clearing of Roboteq errors.
   *   4. Confirm the E-Stop reset was successful with the ReadEStopState method.
   *
   * @throws EStopResetInterrupted if the E-stop reset operation was halted because the E-stop was
   *         triggered again.
   * @throws std::runtime_error if an error occurs when trying to reset the E-stop using GPIO.
   */
  void ResetEStop() override;

  std::shared_ptr<const EStopDiagnostics> GetDiagnostics() const override { return diagnostics_; }

protected:
  void ResetEStopImpl();

  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;
  std::shared_ptr<RobotDriverInterface> robot_driver_;
  std::shared_ptr<std::mutex> robot_driver_write_mtx_;

  std::function<bool()> ZeroVelocityCheck;

  std::mutex e_stop_manipulation_mtx_;
  // Counts software triggers, so a reset that waited for the mutex can tell one happened meanwhile.
  std::atomic<std::uint64_t> trigger_count_{0};
  // Last state seen on the hardware line, so the read loop can report the edge
  // instead of the level - it runs at 100 Hz.
  bool hardware_line_latched_ = true;
  // Set by TriggerEStop until the line edge it causes is reported or the
  // E-stop is reset, so the edge is not blamed on an outside device.
  bool software_trigger_requested_ = false;
  std::atomic_bool e_stop_triggered_ = true;
  std::atomic_bool software_e_stop_triggered_ = true;

  std::shared_ptr<EStopDiagnostics> diagnostics_ = std::make_shared<EStopDiagnostics>();
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_
