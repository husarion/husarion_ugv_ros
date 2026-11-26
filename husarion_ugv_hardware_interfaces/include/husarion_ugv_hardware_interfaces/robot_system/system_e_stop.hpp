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
  // fault or the safety board watchdog. Seen only as an edge of a released line, so a button
  // pressed on a robot that is already latched keeps the earlier source.
  HARDWARE,
  // Latched by the battery undervoltage protection.
  UVP,
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
  /**
   * @brief Triggers the emergency stop and refuses every later reset, for the teardown.
   */
  virtual void Close() = 0;

  /**
   * @brief Triggers the emergency stop for the battery undervoltage protection, which then shows as
   * its cause also when the e-stop was latched already. Called from the control loop, so it must
   * not wait for a reset in progress.
   *
   * @return false when the trigger could not be applied yet, the caller tries again later.
   */
  virtual bool TriggerEStopFromUVP() = 0;
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
    std::shared_ptr<std::mutex> robot_driver_write_mtx, std::function<bool()> zero_velocity_check,
    std::function<bool()> roboteq_e_stop_set = {}, std::function<void()> before_relatch = {})
  : EStopInterface(),
    gpio_controller_(gpio_controller),
    roboteq_error_filter_(roboteq_error_filter),
    robot_driver_(robot_driver),
    robot_driver_write_mtx_(robot_driver_write_mtx),
    ZeroVelocityCheck(zero_velocity_check),
    RoboteqEStopSet(roboteq_e_stop_set),
    BeforeRelatch(before_relatch) {};

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
   *   1. Release the E-stop of the motor controllers.
   *   2. Wait for any trigger or state read in progress, and refuse if a trigger was requested
   *      meanwhile or has not latched the line yet.
   *   3. Verify that the last velocity commands are zero to prevent an uncontrolled robot movement
   *      after an E-stop state change.
   *   4. Attempt to reset the E-Stop using GPIO by manipulating the ESTOP GPIO pin. This operation
   *      may take some time and can be interrupted by the E-Stop trigger process.
   *   5. Refuse if a trigger happened during the GPIO reset.
   *   6. Set the clear_error flag to allow for clearing of Roboteq errors.
   *
   * A refused reset of a latched E-stop latches the E-stop of the motor controllers again, so it
   * leaves their torque disabled until the next successful reset.
   *
   * @throws std::runtime_error if the reset is refused, was halted because the E-stop was
   *         triggered again, or an error occurs when trying to reset the E-stop using GPIO.
   */
  void ResetEStop() override;

  /**
   * @brief Triggers the E-stop and refuses every reset from then on. The teardown calls it before
   * it stops the GPIO controller, so a reset still in flight cannot turn the watchdog on again.
   *
   * @throws std::runtime_error if triggering the E-stop using GPIO fails.
   */
  void Close() override;

  /**
   * @brief Triggers the emergency stop without waiting for the E-stop mutex.
   *
   * The trigger is requested and a GPIO reset in progress is interrupted first, as in TriggerEStop.
   * When the mutex is held, false is returned instead of blocking the control loop. The trigger
   * then stays pending: ReadEStopState() reports the E-stop latched, a reset that holds the mutex
   * applies the trigger and is refused, and so is any later reset until the trigger is applied.
   *
   * @return true when the E-stop is triggered, false when the mutex is held.
   * @throws std::runtime_error if triggering the E-stop using GPIO fails.
   */
  bool TriggerEStopFromUVP() override;

  std::shared_ptr<const EStopDiagnostics> GetDiagnostics() const override { return diagnostics_; }

protected:
  void ResetEStopImpl(const std::uint64_t triggers_before);
  // True when a trigger was requested after triggers_before was read, or one is not applied yet.
  bool TriggerSince(const std::uint64_t triggers_before) const;
  // True while a trigger is requested and not applied yet.
  bool TriggerPending() const;
  // The part of a trigger that runs with e_stop_manipulation_mtx_ held.
  // Returns true when this trigger latched it.
  bool TriggerEStopLocked();
  // Applies the UVP triggers that are not applied yet. Returns true when that latched it.
  bool ApplyUVPTriggersLocked();

  std::shared_ptr<GPIOControllerInterface> gpio_controller_;
  std::shared_ptr<RoboteqErrorFilter> roboteq_error_filter_;
  std::shared_ptr<RobotDriverInterface> robot_driver_;
  std::shared_ptr<std::mutex> robot_driver_write_mtx_;

  std::function<bool()> ZeroVelocityCheck;
  // Whether a motor controller reports its emergency stop fault, read from its driver state.
  std::function<bool()> RoboteqEStopSet;
  // Called before a refused reset latches the motor controllers again, so the diagnostics know
  // the emergency stop fault it causes before the controllers report it.
  std::function<void()> BeforeRelatch;

  std::mutex e_stop_manipulation_mtx_;
  // Software triggers, counted when requested and again once they hold the mutex, so a reset can
  // tell that one came after it started or is still waiting to latch the line.
  std::atomic<std::uint64_t> triggers_requested_{0};
  std::atomic<std::uint64_t> triggers_applied_{0};
  std::atomic_bool closed_ = false;
  // UVP triggers requested and not applied, counted in triggers_requested_ only.
  std::atomic<std::uint64_t> uvp_triggers_unapplied_{0};
  // The read path logs the first failed GPIO write of a streak of UVP applies.
  std::atomic_bool uvp_apply_error_logged_{false};
  // Last state seen on the hardware line, guarded by e_stop_manipulation_mtx_ like the next one.
  bool hardware_line_latched_ = true;
  // Set by TriggerEStop until the line edge it causes is reported or the E-stop is reset, so that
  // edge is not blamed on an outside device.
  bool software_trigger_requested_ = false;
  std::atomic_bool e_stop_triggered_ = true;
  std::atomic_bool software_e_stop_triggered_ = true;

  std::shared_ptr<EStopDiagnostics> diagnostics_ = std::make_shared<EStopDiagnostics>();
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_E_STOP_HPP_
