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

#include <chrono>
#include <memory>
#include <string>

#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"

namespace husarion_ugv_hardware_interfaces
{

bool EStop::ReadEStopState()
{
  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
    e_stop_triggered_ = !gpio_controller_->IsPinActive(GPIOPin::E_STOP_RESET);

    // In the case where E-Stop is triggered by another device within the robot's system (e.g.,
    // Roboteq or Safety Board), disabling the software Watchdog is necessary to prevent an
    // uncontrolled reset.
    if (e_stop_triggered_) {
      software_e_stop_triggered_ = true;
      gpio_controller_->EStopTrigger();
    }
  }

  return e_stop_triggered_ || software_e_stop_triggered_;
}

void EStop::TriggerEStop()
{
  triggers_requested_++;
  gpio_controller_->InterruptEStopReset();

  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  // Counted as applied under the mutex, so a reset holding it sees a trigger still waiting for it.
  triggers_applied_++;
  try {
    software_e_stop_triggered_ = true;
    gpio_controller_->EStopTrigger();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Setting E-stop failed: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;
}

void EStop::Close()
{
  closed_ = true;
  TriggerEStop();
}

void EStop::ResetEStop()
{
  const auto triggers_before = triggers_requested_.load();
  // Decided when the reset starts, so a trigger that refuses it cannot change it.
  const bool latched = e_stop_triggered_ || software_e_stop_triggered_;
  if (closed_) {
    throw std::runtime_error("E-stop reset refused: the hardware interface is shutting down.");
  }

  // Whether the motor controllers were latched before the release, so a refused reset latches
  // them again.
  bool roboteq_latched = latched;
  try {
    // Released as on ros2-devel, under the write mutex like MotorTorqueEnable. Skipped only when
    // the e-stop is released and the motor controllers report no emergency stop: then the control
    // loop is driving and needs the mutex, and there is nothing to release.
    roboteq_latched = latched || (RoboteqEStopSet && RoboteqEStopSet());
    if (roboteq_latched) {
      std::lock_guard<std::mutex> write_lck(*robot_driver_write_mtx_);
      robot_driver_->TurnOffEStop();
    }
    ResetEStopImpl(triggers_before);
  } catch (const std::exception & e) {
    std::string message = e.what();
    // The motor controllers may have been released above, possibly only some of them.
    if (roboteq_latched) {
      try {
        std::lock_guard<std::mutex> write_lck(*robot_driver_write_mtx_);
        robot_driver_->TurnOnEStop();
      } catch (const std::exception & latch_error) {
        message += " Latching the motor controllers again failed: " +
                   std::string(latch_error.what());
      }
    }
    throw std::runtime_error(message);
  }
}

bool EStop::TriggerSince(const std::uint64_t triggers_before) const
{
  const auto requested = triggers_requested_.load();
  return requested != triggers_before || requested != triggers_applied_.load();
}

void EStop::ResetEStopImpl(const std::uint64_t triggers_before)
{
  // Wait for the mutex instead of trying it once: ReadEStopState() takes it briefly on every read
  // cycle. A trigger requested after the reset started, or one not yet applied, wins over it.
  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  // Close() sets this before it waits for the mutex, so no reset releases the line after it.
  if (closed_) {
    throw std::runtime_error("E-stop reset refused: the hardware interface is shutting down.");
  }
  if (TriggerSince(triggers_before)) {
    throw std::runtime_error(
      "E-stop reset refused: the E-stop was triggered while the reset was pending (trigger "
      "request or motor controller communication error).");
  }

  if (!ZeroVelocityCheck()) {
    throw std::runtime_error(
      "Cannot reset the E-stop: the last velocity commands are not zero. Ensure that the "
      "controller is sending zero velocity commands or not sending any velocity commands before "
      "attempting to reset the E-stop.");
  }

  try {
    gpio_controller_->EStopReset();
  } catch (const EStopResetInterrupted & e) {
    throw std::runtime_error(
      "The E-stop reset operation was halted because the E-stop was triggered again.");
  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Error when trying to reset E-stop using GPIO: " + std::string(e.what()));
  }

  // The GPIO reset left the line released with the watchdog running. The trigger that refuses it
  // here waits for this mutex and turns the watchdog off again as soon as it is released.
  if (TriggerSince(triggers_before)) {
    throw std::runtime_error("E-stop reset refused: the E-stop was triggered during the reset.");
  }

  roboteq_error_filter_->SetClearErrorsFlag();

  e_stop_triggered_ = false;
  software_e_stop_triggered_ = false;
}

}  // namespace husarion_ugv_hardware_interfaces
