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

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"

#include <pthread.h>

#include <algorithm>
#include <chrono>
#include <cstring>
#include <ctime>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include "gpiod.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/gpio/types.hpp"
#include "husarion_ugv_utils/configure_rt.hpp"

namespace husarion_ugv_hardware_interfaces
{

Watchdog::PIMutex::PIMutex()
{
  pthread_mutexattr_t attr;
  pthread_mutexattr_init(&attr);
  pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
  const int result = pthread_mutex_init(&mtx_, &attr);
  pthread_mutexattr_destroy(&attr);
  if (result != 0) {
    throw std::runtime_error("Can't create the watchdog mutex: " + std::string(strerror(result)));
  }
}

Watchdog::PIMutex::~PIMutex() { pthread_mutex_destroy(&mtx_); }

void Watchdog::PIMutex::lock() { pthread_mutex_lock(&mtx_); }

void Watchdog::PIMutex::unlock() { pthread_mutex_unlock(&mtx_); }

Watchdog::Watchdog(
  std::shared_ptr<GPIODriverInterface> gpio_driver, const unsigned sched_priority, const int cpu,
  const std::chrono::milliseconds period)
: gpio_driver_(std::move(gpio_driver)), sched_priority_(sched_priority), cpu_(cpu), period_(period)
{
  if (!gpio_driver_->IsPinAvailable(watchdog_pin_)) {
    throw std::runtime_error("Watchdog pin is not configured.");
  }

  // steady_clock is CLOCK_MONOTONIC, so the heartbeat deadlines need a condition on that clock.
  pthread_condattr_t attr;
  pthread_condattr_init(&attr);
  pthread_condattr_setclock(&attr, CLOCK_MONOTONIC);
  pthread_cond_init(&cv_, &attr);
  pthread_condattr_destroy(&attr);

  watchdog_thread_ = std::thread(&Watchdog::WatchdogThread, this);
}

Watchdog::~Watchdog()
{
  {
    std::lock_guard<PIMutex> lck(mtx_);
    shutdown_ = true;
    enabled_ = false;
  }
  NotifyAll();
  watchdog_thread_.join();
  pthread_cond_destroy(&cv_);
}

bool Watchdog::TurnOn()
{
  if (closed_) {
    return false;
  }
  if (IsWatchdogEnabled()) {
    return true;
  }

  if (!gpio_driver_->IsPinAvailable(watchdog_pin_)) {
    throw std::runtime_error("Watchdog pin is not configured.");
  }

  {
    std::lock_guard<PIMutex> lck(mtx_);
    if (closed_) {
      return false;
    }
    enabled_ = true;
  }
  NotifyAll();

  return IsWatchdogEnabled();
}

bool Watchdog::TurnOff()
{
  // Called on every read cycle while the e-stop is latched, so an idle watchdog costs no lock.
  if (!enabled_ && !heartbeat_running_) {
    return true;
  }

  std::unique_lock<PIMutex> lck(mtx_);
  return StopHeartbeat(lck);
}

void Watchdog::Close()
{
  std::unique_lock<PIMutex> lck(mtx_);
  closed_ = true;
  StopHeartbeat(lck);
}

bool Watchdog::StopHeartbeat(std::unique_lock<PIMutex> & lck)
{
  enabled_ = false;
  NotifyAll();
  // Returns only once the heartbeat has stopped and driven the pin low, as the join did when the
  // thread lived only while the watchdog was on. The heartbeat wakes at once instead of finishing
  // its period.
  const auto stops = heartbeat_stops_;
  while (heartbeat_running_ && heartbeat_stops_ == stops) {
    Wait(lck);
  }

  return !IsWatchdogEnabled();
}

void Watchdog::Wait(std::unique_lock<PIMutex> & lck)
{
  pthread_cond_wait(&cv_, lck.mutex()->native());
}

void Watchdog::WaitUntil(
  std::unique_lock<PIMutex> & lck, const std::chrono::steady_clock::time_point & deadline)
{
  const auto ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(deadline.time_since_epoch()).count();
  timespec ts;
  ts.tv_sec = static_cast<std::time_t>(ns / 1000000000LL);
  ts.tv_nsec = static_cast<long>(ns % 1000000000LL);
  pthread_cond_timedwait(&cv_, lck.mutex()->native(), &ts);
}

void Watchdog::NotifyAll() { pthread_cond_broadcast(&cv_); }

void Watchdog::WatchdogThread()
{
  // The safety board watches this square wave and latches the e-stop in hardware when it stops,
  // so a thread held off the core for a few periods stops the robot with nothing in the log.
  pthread_setname_np(pthread_self(), "gpio-wdog");
  try {
    husarion_ugv_utils::ConfigureRT(sched_priority_, cpu_);
  } catch (const std::runtime_error & e) {
    std::cerr << "Failed to configure RT priority for the safety watchdog thread: " << e.what()
              << std::endl;
  }

  std::unique_lock<PIMutex> lck(mtx_);
  while (true) {
    while (!enabled_ && !shutdown_) {
      Wait(lck);
    }
    if (shutdown_) {
      return;
    }

    heartbeat_running_ = true;

    // Absolute deadlines keep the period whatever the toggle costs, and waiting on the condition
    // instead of sleeping lets TurnOff stop the heartbeat at once rather than up to a period later.
    // After a missed deadline the next edge comes at once and the schedule restarts from it.
    auto deadline = std::chrono::steady_clock::now();
    while (enabled_ && !shutdown_) {
      lck.unlock();
      const bool value = gpio_driver_->IsPinActive(watchdog_pin_);
      gpio_driver_->SetPinValue(watchdog_pin_, !value);
      lck.lock();

      deadline = std::max(deadline + period_, std::chrono::steady_clock::now());
      while (enabled_ && !shutdown_ && std::chrono::steady_clock::now() < deadline) {
        WaitUntil(lck, deadline);
      }
    }

    lck.unlock();
    gpio_driver_->SetPinValue(watchdog_pin_, false);
    lck.lock();

    heartbeat_running_ = false;
    ++heartbeat_stops_;
    NotifyAll();
  }
}

bool Watchdog::IsWatchdogEnabled() const { return enabled_; }

void GPIOControllerInterface::Stop() { gpio_driver_->GPIOMonitorDisable(); }

void GPIOControllerInterface::RegisterGPIOEventCallback(
  const std::function<void(const GPIOInfo &)> & callback)
{
  if (!gpio_driver_) {
    throw std::runtime_error("GPIO driver has not been initialized yet.");
  }

  gpio_driver_->ConfigureEdgeEventCallback(callback);
}

bool GPIOControllerInterface::IsPinActive(const GPIOPin pin) const
{
  return gpio_driver_->IsPinActive(pin);
}

bool GPIOControllerInterface::IsPinAvailable(const GPIOPin pin) const
{
  return gpio_driver_->IsPinAvailable(pin);
}

GPIOController::GPIOController(
  std::shared_ptr<GPIODriverInterface> gpio_driver, const unsigned watchdog_sched_priority,
  const unsigned monitor_sched_priority, const int watchdog_cpu, const int monitor_cpu)
: watchdog_sched_priority_(watchdog_sched_priority),
  monitor_sched_priority_(monitor_sched_priority),
  watchdog_cpu_(watchdog_cpu),
  monitor_cpu_(monitor_cpu)
{
  gpio_driver_ = gpio_driver;

  if (!gpio_driver_) {
    throw std::runtime_error("GPIO driver is not initialized.");
  }
}

void GPIOController::Start()
{
  gpio_driver_->GPIOMonitorEnable(true, monitor_sched_priority_, monitor_cpu_);

  gpio_driver_->SetPinValue(GPIOPin::VMOT_ON, true);
  MotorPowerEnable(true);

  watchdog_ = std::make_unique<Watchdog>(gpio_driver_, watchdog_sched_priority_, watchdog_cpu_);
}

void GPIOController::Stop()
{
  // For good, so nothing turns it on again for the rest of the teardown. EStop::Close() already
  // refuses resets; this covers a caller that does not go through the e-stop.
  if (watchdog_) {
    watchdog_->Close();
  }
  GPIOControllerInterface::Stop();
}

void GPIOController::EStopTrigger()
{
  if (!watchdog_->TurnOff()) {
    throw std::runtime_error("Can't stop watchdog thread");
  }
}

void GPIOController::EStopReset()
{
  const auto e_stop_pin = GPIOPin::E_STOP_RESET;
  bool e_stop_state = !gpio_driver_->IsPinActive(e_stop_pin);

  if (!watchdog_->TurnOn()) {
    throw std::runtime_error("The watchdog is closed, the GPIO controller is stopping.");
  }

  if (!e_stop_state) {
    fprintf(stdout, "[GPIOController] E-stop is not active, reset is not needed\n");
    return;
  }

  gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::OUTPUT);
  gpio_driver_->SetPinValue(e_stop_pin, true);

  if (!WaitFor(std::chrono::milliseconds(100))) {
    gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::INPUT);
    throw EStopResetInterrupted("E-stop reset interrupted after setting pin high.");
  }

  gpio_driver_->ChangePinDirection(e_stop_pin, gpiod::line::direction::INPUT);

  if (!WaitFor(std::chrono::milliseconds(100))) {
    throw EStopResetInterrupted("E-stop reset interrupted after setting pin high.");
  }

  e_stop_state = !gpio_driver_->IsPinActive(e_stop_pin);

  if (e_stop_state) {
    watchdog_->TurnOff();
    throw std::runtime_error(
      "E-stop reset failed, check for pressed E-stop buttons or other triggers.");
  }
}

bool GPIOController::MotorPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(GPIOPin::DRIVER_EN, enable);
};

bool GPIOController::AUXPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(GPIOPin::AUX_PW_EN, enable);
};

bool GPIOController::FanEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(GPIOPin::FAN_SW, enable);
};

bool GPIOController::DigitalPowerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(GPIOPin::VDIG_OFF, !enable);
};

bool GPIOController::ChargerEnable(const bool enable)
{
  return gpio_driver_->SetPinValue(GPIOPin::CHRG_DISABLE, !enable);
}

bool GPIOController::LEDControlEnable(const bool enable)
{
  // pin_validation_wait_time=10ms used due to slow pin state transition
  // on pin loaded by high 100nF capacity in SBC Overlay v1.4
  return gpio_driver_->SetPinValue(GPIOPin::LED_SBC_SEL, enable, std::chrono::milliseconds(10));
}

std::unordered_map<GPIOPin, bool> GPIOController::QueryControlInterfaceIOStates() const
{
  std::unordered_map<GPIOPin, bool> io_state;

  std::vector<GPIOPin> pins_to_query = {
    GPIOPin::AUX_PW_EN, GPIOPin::CHRG_SENSE, GPIOPin::CHRG_DISABLE, GPIOPin::VDIG_OFF,
    GPIOPin::FAN_SW,    GPIOPin::SHDN_INIT,  GPIOPin::VMOT_ON,
  };

  std::for_each(pins_to_query.begin(), pins_to_query.end(), [&](GPIOPin pin) {
    bool is_active = gpio_driver_->IsPinActive(pin);
    io_state.emplace(pin, is_active);
  });

  return io_state;
}

const std::vector<GPIOInfo> GPIOController::gpio_config_info_storage_ = {
  GPIOInfo{GPIOPin::WATCHDOG, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
  GPIOInfo{
    GPIOPin::CHRG_DISABLE, gpiod::line::direction::OUTPUT, false, gpiod::line::value::ACTIVE},
  GPIOInfo{GPIOPin::DRIVER_EN, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::E_STOP_RESET, gpiod::line::direction::INPUT},
  GPIOInfo{GPIOPin::FAN_SW, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::GPOUT1, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::GPOUT2, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::GPIN1, gpiod::line::direction::INPUT},
  GPIOInfo{GPIOPin::GPIN2, gpiod::line::direction::INPUT},
  GPIOInfo{GPIOPin::SHDN_INIT, gpiod::line::direction::INPUT},
  GPIOInfo{GPIOPin::VDIG_OFF, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::VMOT_ON, gpiod::line::direction::OUTPUT},
  GPIOInfo{GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT, true},
  GPIOInfo{GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT, true},
};

const std::vector<GPIOInfo> & GPIOController::GetGPIOConfigInfoStorage()
{
  return gpio_config_info_storage_;
}

void GPIOController::InterruptEStopReset()
{
  std::lock_guard<std::mutex> lck(e_stop_cv_mtx_);
  should_abort_e_stop_reset_ = true;
  e_stop_cv_.notify_one();
}

bool GPIOController::WaitFor(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lck(e_stop_cv_mtx_);

  should_abort_e_stop_reset_ = false;
  bool interrupted = e_stop_cv_.wait_for(
    lck, timeout, [&]() { return should_abort_e_stop_reset_; });

  return !interrupted;
}

}  // namespace husarion_ugv_hardware_interfaces
