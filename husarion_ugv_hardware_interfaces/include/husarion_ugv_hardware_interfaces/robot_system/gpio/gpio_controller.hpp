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

/**
 * @file gpio_controller.hpp
 * @brief Header file containing a higher-level wrapper for the GPIO driver.
 */

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_CONTROLLER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_CONTROLLER_HPP_

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

#include "gpiod.hpp"

#include "husarion_ugv_utils/common_utilities.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/gpio/types.hpp"

namespace husarion_ugv_hardware_interfaces
{

class EStopResetInterrupted : public std::exception
{
public:
  EStopResetInterrupted(const std::string & message) : msg_(message) {}
  const char * what() const noexcept override { return msg_.c_str(); }

private:
  std::string msg_;
};

class Watchdog
{
public:
  /**
   * @brief Constructor for Watchdog class.
   *
   * @param gpio_driver Pointer to the GPIODriver object.
   * @param sched_priority SCHED_FIFO priority of the watchdog thread.
   * @param cpu CPU the watchdog thread pins itself to, -1 to not pin.
   * @param period Time between two edges of the heartbeat.
   * @exception std::runtime_error if the Watchdog pin is not configured by GPIODriver or not
   * described in GPIOController gpio_info storage
   */
  Watchdog(
    std::shared_ptr<GPIODriverInterface> gpio_driver,
    const unsigned sched_priority = kWatchdogSchedPriority, const int cpu = -1,
    const std::chrono::milliseconds period = kPeriod);

  /**
   * @brief Destructor for Watchdog class. Stops the heartbeat and joins the watchdog thread.
   */
  ~Watchdog();

  /**
   * @brief Turns on the Watchdog.
   *
   * @return True if successful, false after Close().
   * @exception std::runtime_error if the Watchdog pin is not configured by GPIODriver or not
   * described in GPIOController gpio_info storage
   */
  bool TurnOn();

  /**
   * @brief Turns off the Watchdog. Returns once the heartbeat has stopped and the pin is low.
   * TurnOn and TurnOff are serialized by the e-stop mutex. Close() is the exception: it takes only
   * the watchdog mutex, which is enough because it waits for the heartbeat itself and every later
   * TurnOn refuses.
   */
  bool TurnOff();

  /**
   * @brief Turns the Watchdog off for good: TurnOn refuses from then on. The teardown calls it.
   */
  void Close();

  bool IsWatchdogEnabled() const;

  // Feeding the safety board is the most timing-critical thing this process does, so it runs above
  // the control loop.
  static constexpr unsigned kWatchdogSchedPriority = 60;
  static constexpr std::chrono::milliseconds kPeriod{10};

private:
  // A priority inheriting mutex: the e-stop services that take it can run SCHED_OTHER, and one of
  // them preempted while holding it must not hold the heartbeat off.
  class PIMutex
  {
  public:
    PIMutex();
    ~PIMutex();
    void lock();
    void unlock();
    pthread_mutex_t * native() { return &mtx_; }

  private:
    pthread_mutex_t mtx_;
  };

  /**
   * @brief Body of the watchdog thread.
   *
   * While the Watchdog is on, it toggles the state of the Watchdog pin between active and
   * inactive states every period. When it is turned off, it sets the Watchdog pin to an inactive
   * state and parks until it is turned on again or destroyed.
   */
  void WatchdogThread();
  bool StopHeartbeat(std::unique_lock<PIMutex> & lck);
  void Wait(std::unique_lock<PIMutex> & lck);
  void WaitUntil(
    std::unique_lock<PIMutex> & lck, const std::chrono::steady_clock::time_point & deadline);
  void NotifyAll();

  GPIOPin watchdog_pin_ = GPIOPin::WATCHDOG;
  std::shared_ptr<GPIODriverInterface> gpio_driver_;
  const unsigned sched_priority_;
  const int cpu_;
  const std::chrono::milliseconds period_;

  // One thread for the life of the Watchdog, parked while it is off, so its priority and CPU
  // survive e-stop resets. The flags are written under mtx_ and read lock-free on the fast path.
  PIMutex mtx_;
  pthread_cond_t cv_;
  std::atomic_bool enabled_ = false;
  std::atomic_bool shutdown_ = false;
  std::atomic_bool closed_ = false;
  std::atomic_bool heartbeat_running_ = false;
  std::uint64_t heartbeat_stops_ = 0;  // guarded by mtx_
  std::thread watchdog_thread_;
};

class GPIOControllerInterface
{
public:
  virtual ~GPIOControllerInterface() = default;

  virtual void Start() = 0;

  /**
   * @brief Stops the GPIO monitor thread, so no GPIO event callback runs from it after this
   * returns. SetPinValue() still runs the callback on the caller's thread.
   */
  virtual void Stop();

  virtual void EStopTrigger() = 0;
  virtual void EStopReset() = 0;

  virtual bool MotorPowerEnable(const bool enable) = 0;
  virtual bool FanEnable(const bool enable) = 0;
  virtual bool AUXPowerEnable(const bool enable) = 0;
  virtual bool DigitalPowerEnable(const bool enable) = 0;
  virtual bool ChargerEnable(const bool enable) = 0;
  virtual bool LEDControlEnable(const bool enable) = 0;

  virtual std::unordered_map<GPIOPin, bool> QueryControlInterfaceIOStates() const = 0;

  virtual void InterruptEStopReset() {};

  /**
   * @brief This method sets the provided callback function to be executed upon GPIO edge events.
   *
   * @param callback The callback function to handle GPIO edge events.
   *
   * @par Example
   * An example of using this method to bind a member function as a callback:
   * @code{.cpp}
   * class MyClass {
   * public:
   *   void HandleGPIOEvent(const GPIOInfo & gpio_info) {
   *     // Handle GPIO event here, i.e:
   *     std::cout << gpio_info.offset << ":    " << gpio_info.value << std::endl;
   *   }
   * };
   *
   * MyClass my_obj;
   * GPIOController gpio_controller;
   * gpio_controller.RegisterGPIOEventCallback(
   *     std::bind(&MyClass::HandleGPIOEvent, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void RegisterGPIOEventCallback(const std::function<void(const GPIOInfo &)> & callback);

  bool IsPinActive(const GPIOPin pin) const;

  bool IsPinAvailable(const GPIOPin pin) const;

protected:
  std::shared_ptr<GPIODriverInterface> gpio_driver_;
};

class GPIOController : public GPIOControllerInterface
{
public:
  /**
   * @brief Constructor for GPIOController class.
   *
   * @param gpio_driver Pointer to the GPIODriver object.
   * @param watchdog_sched_priority SCHED_FIFO priority of the safety watchdog thread.
   * @param monitor_sched_priority SCHED_FIFO priority of the GPIO monitor thread.
   * @param watchdog_cpu CPU the safety watchdog thread pins itself to, -1 to not pin.
   * @param monitor_cpu CPU the GPIO monitor thread pins itself to, -1 to not pin.
   * @throw `std::runtime_error` When the GPIO driver is not initialized.
   */
  GPIOController(
    std::shared_ptr<GPIODriverInterface> gpio_driver,
    const unsigned watchdog_sched_priority = Watchdog::kWatchdogSchedPriority,
    const unsigned monitor_sched_priority = kMonitorSchedPriority, const int watchdog_cpu = -1,
    const int monitor_cpu = -1);

  static constexpr unsigned kMonitorSchedPriority = 60;

  /**
   * @brief Initializes the GPIODriver, Watchdog, and powers on the motors.
   */
  void Start() override;

  /**
   * @brief Turns off the Watchdog, which reads pins through the monitor, then stops the GPIO
   * monitor thread.
   */
  void Stop() override;

  /**
   * @brief Disables the Watchdog thread for E-stop mechanism trigger.
   *
   * @return true if the Watchdog thread is successfully disabled.
   * @exception std::runtime_error when the Watchdog thread fails to stop.
   */
  void EStopTrigger() override;

  /**
   * @brief Resets the E-stop.
   *
   * This method verifies the status of the E_STOP_RESET pin, which is configured as an input.
   * If the pin is active, it attempts to reset the E-stop by momentarily setting it to an
   * inactive state. During this reset process, the pin is configured as an output for a specific
   * duration. If the attempt to reset the E-stop fails (the pin reads its value as an input
   * again), it throws a runtime error. The Watchdog thread is temporarily activated during the
   * E-stop reset process.
   * @return true if the E-stop is successfully reset.
   * @exception std::runtime_error when the E-stop reset fails.
   */
  void EStopReset() override;

  /**
   * @brief Controls the motor power by enabling or disabling them based on the 'enable'
   * parameter.
   *
   * @param enable Set to 'true' to enable the motors, 'false' to disable.
   * @return 'true' if the motor control pin value is successfully set, 'false' otherwise.
   */
  bool MotorPowerEnable(const bool enable) override;

  /**
   * @brief Controls the fan based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the fan, 'false' to disable.
   * @return 'true' if the fan control pin value is successfully set, 'false' otherwise.
   */
  bool FanEnable(const bool enable) override;

  /**
   * @brief Controls AUX power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the AUX power, 'false' to disable.
   * @return 'true' if the AUX control pin value is successfully set, 'false' otherwise.
   */
  bool AUXPowerEnable(const bool enable) override;

  /**
   * @brief Controls the digital power source based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the VDIG, 'false' to disable.
   * @return 'true' if the VDIG control pin value is successfully set, 'false' otherwise.
   */
  bool DigitalPowerEnable(const bool enable) override;

  /**
   * @brief Enables or disables the use of an external charger according to the 'enable'
   * parameter.
   *
   * @param enable Set to 'true' to enable external charger, 'false' to disable.
   * @return 'true' if the charger control pin value is successfully set, 'false' otherwise.
   */
  bool ChargerEnable(const bool enable) override;

  /**
   * @brief Enables or disables the LED control based on the 'enable' parameter.
   *
   * @param enable Set to 'true' to enable the LED control, 'false' to disable.
   * @return 'true' if the LED control pin value is successfully set, 'false' otherwise.
   */
  bool LEDControlEnable(const bool enable) override;

  /**
   * @brief Queries the current IO states of the control interface.
   *
   * @return An unordered map containing the GPIOPin as the key and its active state as the value.
   */
  std::unordered_map<GPIOPin, bool> QueryControlInterfaceIOStates() const override;

  /**
   * @brief Returns the GPIO pin configuration information for the PTH12X.
   */
  static const std::vector<GPIOInfo> & GetGPIOConfigInfoStorage();

  void InterruptEStopReset() override;

protected:
  std::unique_ptr<Watchdog> watchdog_;

private:
  const unsigned watchdog_sched_priority_;
  const unsigned monitor_sched_priority_;
  const int watchdog_cpu_;
  const int monitor_cpu_;

  /**
   * @brief Waits for a specific duration or until an interruption is signaled.
   *
   * This method is designed to block execution for the specified timeout duration. It also
   * monitors for an interruption signal which, if received, will cause the method to return
   * early. The interruption is controlled by the `should_abort_e_stop_reset_` flag.
   *
   * @param timeout Duration to wait for in milliseconds.
   * @return `true` if the wait completed without interruption, `false` if an interruption was
   * signaled.
   */
  bool WaitFor(std::chrono::milliseconds timeout);

  /**
   * @brief Vector containing GPIO pin configuration information such as pin direction, value,
   * etc.
   */
  static const std::vector<GPIOInfo> gpio_config_info_storage_;

  std::mutex e_stop_cv_mtx_;
  std::condition_variable e_stop_cv_;
  volatile std::atomic_bool should_abort_e_stop_reset_ = false;
};

class GPIOControllerFactory
{
public:
  /**
   * @brief Creates a GPIO controller.
   *
   * @param watchdog_sched_priority SCHED_FIFO priority of the safety watchdog thread.
   * @param monitor_sched_priority SCHED_FIFO priority of the GPIO monitor thread.
   * @param watchdog_cpu CPU the safety watchdog thread pins itself to, -1 to not pin.
   * @param monitor_cpu CPU the GPIO monitor thread pins itself to, -1 to not pin.
   * @return A unique pointer to the created GPIO controller.
   */
  static std::unique_ptr<GPIOControllerInterface> CreateGPIOController(
    const unsigned watchdog_sched_priority = Watchdog::kWatchdogSchedPriority,
    const unsigned monitor_sched_priority = GPIOController::kMonitorSchedPriority,
    const int watchdog_cpu = -1, const int monitor_cpu = -1)
  {
    std::unique_ptr<GPIOControllerInterface> gpio_controller;
    auto config_info_storage = GPIOController::GetGPIOConfigInfoStorage();
    auto gpio_driver = std::make_shared<GPIODriver>(config_info_storage);

    gpio_controller = std::make_unique<GPIOController>(
      gpio_driver, watchdog_sched_priority, monitor_sched_priority, watchdog_cpu, monitor_cpu);

    return gpio_controller;
  };
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_CONTROLLER_HPP_
