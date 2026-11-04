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
#include <cstdio>
#include <iostream>
#include <memory>
#include <string>

#include "husarion_ugv_hardware_interfaces/robot_system/system_e_stop.hpp"

namespace husarion_ugv_hardware_interfaces
{

void EStopDiagnostics::SetSource(const EStopLatchSource source)
{
  source_.store(source);
  last_change_ns_.store(SteadyNowNs());
}

void EStopDiagnostics::SetResetRefused(const std::string & reason)
{
  std::lock_guard<std::mutex> lck(refused_reset_mtx_);
  last_refused_reset_ = reason;
  last_refused_reset_ns_ = SteadyNowNs();
}

EStopDiagnostics::Snapshot EStopDiagnostics::Get() const
{
  Snapshot snapshot;
  snapshot.source = source_.load();
  snapshot.last_change_ns = last_change_ns_.load();

  std::lock_guard<std::mutex> lck(refused_reset_mtx_);
  snapshot.last_refused_reset = last_refused_reset_;
  snapshot.last_refused_reset_ns = last_refused_reset_ns_;
  return snapshot;
}

std::int64_t EStopDiagnostics::SteadyNowNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

bool EStop::ReadEStopState()
{
  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
    e_stop_triggered_ = !gpio_controller_->IsPinActive(GPIOPin::E_STOP_RESET);

    // The hardware line is the only evidence that something outside this
    // process stopped the robot, and it used to change state without a trace -
    // a pressed button, a Roboteq fault and a safety board watchdog timeout all
    // read as silence in the log, which cost an afternoon of forensics. Report
    // the transition, once per edge.
    if (e_stop_triggered_ != hardware_line_latched_) {
      hardware_line_latched_ = e_stop_triggered_;
      // A software trigger stops the watchdog, and the safety board drops the
      // line some tens of milliseconds later. That edge is ours, not an outside
      // device's.
      if (hardware_line_latched_ && software_trigger_requested_) {
        software_trigger_requested_ = false;
        std::cerr << "E-Stop ASSERTED on the hardware line after the software trigger requested "
                     "by this driver."
                  << std::endl;
      } else if (hardware_line_latched_) {
        diagnostics_->SetSource(EStopLatchSource::HARDWARE);
        std::cerr << "E-Stop ASSERTED on the hardware line - stopped by a device outside this "
                     "driver (e-stop button, Roboteq fault or the safety board watchdog). No "
                     "software trigger was requested."
                  << std::endl;
      } else {
        std::cerr << "E-Stop hardware line released." << std::endl;
      }
    }

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
  std::cerr << "E-Stop triggered in software (service call or driver shutdown)." << std::endl;
  trigger_count_++;
  gpio_controller_->InterruptEStopReset();

  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  try {
    software_e_stop_triggered_ = true;
    software_trigger_requested_ = true;
    gpio_controller_->EStopTrigger();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Setting E-stop failed: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;

  // A trigger on a robot that is already stopped keeps the first cause - an outside device that
  // stopped it stays the reason until someone resets it.
  if (diagnostics_->GetSource() == EStopLatchSource::RELEASED) {
    diagnostics_->SetSource(EStopLatchSource::SOFTWARE);
  }
}

void EStop::ResetEStop()
{
  try {
    ResetEStopImpl();
  } catch (const std::exception & e) {
    diagnostics_->SetResetRefused(e.what());
    throw;
  }
}

void EStop::ResetEStopImpl()
{
  robot_driver_->TurnOffEStop();

  // ReadEStopState() takes this mutex on every read cycle, so a reset that gave up at the first
  // busy try lost that race now and then and still reported success (a gate run saw "Resetting
  // E-Stop." with the line never released). Wait for the mutex instead, as TriggerEStop() does,
  // and refuse when a trigger happened while waiting - releasing the line then would undo it.
  const auto triggers_before = trigger_count_.load();
  std::unique_lock<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  if (trigger_count_.load() != triggers_before) {
    throw std::runtime_error("The E-stop was triggered while the reset waited, so it stays set.");
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

  roboteq_error_filter_->SetClearErrorsFlag();

  e_stop_triggered_ = false;
  software_e_stop_triggered_ = false;
  software_trigger_requested_ = false;

  diagnostics_->SetSource(EStopLatchSource::RELEASED);
}

}  // namespace husarion_ugv_hardware_interfaces
