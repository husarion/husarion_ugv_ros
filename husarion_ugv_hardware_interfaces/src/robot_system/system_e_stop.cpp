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
  const char * edge_message = nullptr;
  std::string uvp_apply_error;

  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);
    e_stop_triggered_ = !gpio_controller_->IsPinActive(GPIOPin::E_STOP_RESET);

    // The hardware line is the only trace of a stop by something outside this driver. Report each
    // edge once, the read runs at the control loop rate.
    if (e_stop_triggered_ != hardware_line_latched_) {
      hardware_line_latched_ = e_stop_triggered_;
      // A software trigger stops the watchdog, and the safety board drops the line some tens of
      // milliseconds later. That edge is ours.
      if (hardware_line_latched_ && software_trigger_requested_) {
        software_trigger_requested_ = false;
        edge_message =
          "E-Stop ASSERTED on the hardware line after the software trigger requested by this "
          "driver.";
      } else if (hardware_line_latched_) {
        diagnostics_->SetSource(EStopLatchSource::HARDWARE);
        edge_message =
          "E-Stop ASSERTED on the hardware line - stopped by a device outside this driver (e-stop "
          "button, Roboteq fault or the safety board watchdog). No software trigger was requested.";
      } else {
        edge_message = "E-Stop hardware line released.";
      }
    }

    // Defensive: every path that sets RELEASED also clears hardware_line_latched_, so a later latch
    // goes through the edge check above. A source that says RELEASED on a latched line is wrong.
    if (
      hardware_line_latched_ && !software_trigger_requested_ &&
      diagnostics_->GetSource() == EStopLatchSource::RELEASED) {
      diagnostics_->SetSource(EStopLatchSource::HARDWARE);
    }

    // In the case where E-Stop is triggered by another device within the robot's system (e.g.,
    // Roboteq or Safety Board), disabling the software Watchdog is necessary to prevent an
    // uncontrolled reset.
    if (e_stop_triggered_) {
      software_e_stop_triggered_ = true;
      gpio_controller_->EStopTrigger();
    }

    // A UVP trigger that found the mutex held is applied here at the latest, also when the
    // protection cleared before it was tried again. A failed GPIO write leaves it pending, so the
    // state stays latched, and every read tries again; the first failure of a streak is logged.
    try {
      ApplyUVPTriggersLocked();
    } catch (const std::runtime_error & e) {
      if (!uvp_apply_error_logged_.exchange(true)) {
        uvp_apply_error = e.what();
      }
    }
  }

  if (!uvp_apply_error.empty()) {
    std::cerr << "Applying the battery undervoltage E-stop trigger failed, retrying: "
              << uvp_apply_error << std::endl;
  }

  // Written after the mutex is released, so a slow stderr never holds up a trigger or a reset.
  // Edges are rare, keep it that way: this runs on the control loop.
  if (edge_message) {
    std::cerr << edge_message << std::endl;
  }

  // A trigger that is requested but not applied yet, as a UVP trigger that found the mutex held,
  // already counts: the next write sends no motion. Read before the flags: a trigger sets the
  // software flag before it counts itself applied, so a read that no longer sees it pending sees
  // the flag.
  const bool pending = TriggerPending();
  return pending || e_stop_triggered_ || software_e_stop_triggered_;
}

void EStop::TriggerEStop()
{
  triggers_requested_++;
  gpio_controller_->InterruptEStopReset();

  bool newly_triggered = false;
  {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
    // Counted as applied under the mutex, so a reset holding it sees a trigger still waiting, and
    // only once the software flag is set, so a read never sees it neither pending nor latched.
    try {
      newly_triggered = TriggerEStopLocked();
    } catch (const std::runtime_error &) {
      triggers_applied_++;
      throw;
    }
    triggers_applied_++;

    // A trigger on a robot that is already stopped keeps the first cause - an outside device that
    // stopped it stays the reason until someone resets it.
    if (diagnostics_->GetSource() == EStopLatchSource::RELEASED) {
      diagnostics_->SetSource(EStopLatchSource::SOFTWARE);
    }
  }

  // Only the trigger that latches it is logged: a communication error triggers on every cycle.
  if (newly_triggered) {
    std::cerr << "E-Stop triggered in software by this driver." << std::endl;
  }
}

void EStop::Close()
{
  closed_ = true;
  TriggerEStop();
}

bool EStop::TriggerEStopLocked()
{
  bool newly_triggered = false;
  try {
    newly_triggered = !software_e_stop_triggered_;
    software_e_stop_triggered_ = true;
    // Only a trigger on a released line causes the next edge.
    if (!hardware_line_latched_) {
      software_trigger_requested_ = true;
    }
    gpio_controller_->EStopTrigger();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Setting E-stop failed: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;
  return newly_triggered;
}

bool EStop::TriggerEStopFromUVP()
{
  // Requested and not applied until it is, so a reset refuses it and ReadEStopState() reports the
  // e-stop latched from here on, also while a reset or a reader holds the mutex.
  // Counted unapplied before requested, so a reset that sees the request finds it to apply.
  uvp_triggers_unapplied_++;
  triggers_requested_++;
  gpio_controller_->InterruptEStopReset();

  // A GPIO reset can hold the mutex for up to 200 ms, and an interrupt that lands between its two
  // waits is not seen. The control loop must not wait for it: a reset that holds the mutex applies
  // the trigger before it releases it, and otherwise the next cycle tries again.
  std::unique_lock<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::try_to_lock);
  if (!e_stop_lck.owns_lock()) {
    return false;
  }

  const bool newly_triggered = ApplyUVPTriggersLocked();
  e_stop_lck.unlock();

  // The guard repeats the trigger while the battery stays low: log the one that latches it.
  if (newly_triggered) {
    std::cerr << "E-Stop triggered in software by the battery undervoltage protection."
              << std::endl;
  }
  return true;
}

bool EStop::ApplyUVPTriggersLocked()
{
  const auto unapplied = uvp_triggers_unapplied_.exchange(0);
  if (unapplied == 0) {
    return false;
  }

  bool newly_triggered = false;
  try {
    newly_triggered = TriggerEStopLocked();
  } catch (const std::runtime_error &) {
    // Still pending: the software flag is set, and the next trigger tries the GPIO again.
    uvp_triggers_unapplied_ += unapplied;
    throw;
  }
  uvp_apply_error_logged_ = false;
  // Unlike other triggers, this one replaces the first cause: the protection keeps the robot
  // stopped for as long as the battery stays low, whatever latched it before.
  diagnostics_->SetSource(EStopLatchSource::UVP);
  triggers_applied_ += unapplied;
  return newly_triggered;
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
      if (BeforeRelatch) {
        BeforeRelatch();
      }
      try {
        std::lock_guard<std::mutex> write_lck(*robot_driver_write_mtx_);
        robot_driver_->TurnOnEStop();
      } catch (const std::exception & latch_error) {
        message += " Latching the motor controllers again failed: " +
                   std::string(latch_error.what());
      }
    }
    diagnostics_->SetResetRefused(message);
    throw std::runtime_error(message);
  }

  // The reset marks the line released itself, so the read path sees no edge for it.
  std::cerr << "E-Stop reset, hardware line released." << std::endl;
}

bool EStop::TriggerPending() const
{
  return triggers_requested_.load() != triggers_applied_.load();
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
  bool gpio_reset_ran = false;
  try {
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
      gpio_reset_ran = true;
      gpio_controller_->EStopReset();
    } catch (const EStopResetInterrupted & e) {
      throw std::runtime_error(
        "The E-stop reset operation was halted because the E-stop was triggered again.");
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Error when trying to reset E-stop using GPIO: " + std::string(e.what()));
    }

    // The GPIO reset left the line released with the watchdog running. A trigger that refuses it
    // here waits for this mutex and turns the watchdog off again as soon as it is released, or is
    // a UVP trigger, applied below.
    if (TriggerSince(triggers_before)) {
      throw std::runtime_error("E-stop reset refused: the E-stop was triggered during the reset.");
    }
  } catch (const std::exception & e) {
    // A UVP trigger that found the mutex held is applied before the mutex is released, so the
    // line the GPIO reset may have released is latched again at once.
    std::string message = e.what();
    if (gpio_reset_ran && TriggerSince(triggers_before)) {
      // The GPIO reset may have released the line while hardware_line_latched_ still says latched.
      // The edge of the trigger that refused the reset is ours, not an outside device's.
      software_trigger_requested_ = true;
    }
    try {
      ApplyUVPTriggersLocked();
    } catch (const std::exception & trigger_error) {
      message += " Applying the battery undervoltage trigger failed: " +
                 std::string(trigger_error.what());
    }
    throw std::runtime_error(message);
  }

  roboteq_error_filter_->SetClearErrorsFlag();

  e_stop_triggered_ = false;
  software_e_stop_triggered_ = false;
  software_trigger_requested_ = false;
  // EStopReset() saw the line released, so a latch right after this is a new edge.
  hardware_line_latched_ = false;

  diagnostics_->SetSource(EStopLatchSource::RELEASED);
}

}  // namespace husarion_ugv_hardware_interfaces
