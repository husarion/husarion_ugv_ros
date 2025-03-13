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
      gpio_controller_->EStopTrigger();
    }
  }

  return e_stop_triggered_;
}

void EStop::TriggerEStop()
{
  gpio_controller_->InterruptEStopReset();

  std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_);
  try {
    gpio_controller_->EStopTrigger();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Setting E-stop failed: " + std::string(e.what()));
  }

  e_stop_triggered_ = true;
}

void EStop::ResetEStop()
{
  robot_driver_->TurnOffEStop();
  if (e_stop_manipulation_mtx_.try_lock()) {
    std::lock_guard<std::mutex> e_stop_lck(e_stop_manipulation_mtx_, std::adopt_lock);

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
  } else {
    std::fprintf(stdout, "E-stop trigger operation is in progress, skipping reset.");
  }
}

}  // namespace husarion_ugv_hardware_interfaces
