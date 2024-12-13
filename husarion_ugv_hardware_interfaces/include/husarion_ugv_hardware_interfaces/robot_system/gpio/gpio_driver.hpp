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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_DRIVER_HPP_

#include <atomic>
#include <condition_variable>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <vector>

#include "gpiod.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/types.hpp"

namespace husarion_ugv_hardware_interfaces
{

class GPIODriverInterface
{
public:
  virtual ~GPIODriverInterface() = default;
  virtual void GPIOMonitorEnable(
    const bool use_rt = false, const unsigned gpio_monit_thread_sched_priority = 60) = 0;
  virtual void ConfigureEdgeEventCallback(
    const std::function<void(const GPIOInfo &)> & callback) = 0;
  virtual void ChangePinDirection(const GPIOPin pin, const gpiod::line::direction direction) = 0;
  virtual bool IsPinAvailable(const GPIOPin pin) const = 0;
  virtual bool IsPinActive(const GPIOPin pin) = 0;
  virtual bool SetPinValue(
    const GPIOPin pin, const bool value,
    const std::chrono::milliseconds & pin_validation_wait_time = std::chrono::milliseconds(0)) = 0;
};

/**
 * @brief Class responsible for managing GPIO pins on Panther robots, handling tasks such as
 * setting pin values, changing pin directions, monitoring pin events, and more.
 */
class GPIODriver : public GPIODriverInterface
{
public:
  /**
   * @brief Constructs the GPIODriver object with information about GPIO pin configurations.
   * This information is necessary for initializing the GPIO functionality.
   *
   * @param gpio_info_storage Vector containing information about GPIO pin configurations.
   *
   * @throws std::runtime_error if the provided `gpio_info_storage` vector is empty.
   *
   * @note To enable reading pin values, it is required to enable GPIO monitoring.
   * See GPIOMonitorEnable method for more info.
   *
   * @par Example
   * An example of constructing the GPIODriver object by providing GPIO pin information:
   * @code{.cpp}
   * std::vector<GPIOInfo> gpio_configurations = {
   *   GPIOInfo{GPIOPin::CHRG_SENSE, gpiod::line::direction::INPUT},
   *   GPIOInfo{GPIOPin::AUX_PW_EN, gpiod::line::direction::OUTPUT},
   *   GPIOInfo{GPIOPin::LED_SBC_SEL, gpiod::line::direction::OUTPUT, true,
   * gpiod::line::value::ACTIVE}
   *   // ... additional GPIO pin configurations
   * };
   * GPIODriver gpio_driver(gpio_configurations);
   * @endcode
   */
  GPIODriver(std::vector<GPIOInfo> gpio_info_storage);

  /**
   * @brief The destructor sets the GPIO pin values back to their initial values to ensure proper
   * cleanup. It then releases the line request and turns off the GPIO monitoring thread.
   */
  ~GPIODriver();

  /**
   * @brief Enables GPIO pin state monitoring asynchronously. Optionally, configure the
   * Real-Time (RT) FIFO scheduling policy for the monitor thread.
   * @details When called, this method starts the GPIO state monitoring thread asynchronously.
   * If `use_rt` is set to `true`, it configures the FIFO RT scheduling policy for the monitor
   * thread with the specified `gpio_monit_thread_sched_priority`. The default priority is 60.
   *
   * @param use_rt Whether to configure RT FIFO scheduling policy for the monitor thread.
   *        Default is set to false.
   * @param gpio_monit_thread_sched_priority Priority for the GPIO monitoring thread.
   *        Set within the range of 0-99 to enable and configure the FIFO RT scheduling
   *        policy for the monitor thread. This parameter is considered only if `use_rt` is set to
   *        true. The default priority is 60.
   *
   * @note Calling `GPIOMonitorEnable` is optional after constructing the driver object. It allows
   *       asynchronous monitoring of GPIO pin states. Not invoking this method will result in the
   *       lack of functionality to read pin values.
   */
  void GPIOMonitorEnable(
    const bool use_rt = false, const unsigned gpio_monit_thread_sched_priority = 60) override;

  /**
   * @brief This method sets the provided callback function to be executed upon GPIO edge events.
   *
   * @param callback The callback function to handle GPIO edge events.
   *
   * @throws std::runtime_error if GPIO monitor thread is not running.
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
   * GPIODriver gpio_driver;
   * gpio_driver.GPIOMonitorEnable(true, 50);
   * gpio_driver.ConfigureEdgeEventCallback(
   *     std::bind(&MyClass::HandleGPIOEvent, &my_obj, std::placeholders::_1));
   * @endcode
   */
  void ConfigureEdgeEventCallback(const std::function<void(const GPIOInfo &)> & callback) override;

  /**
   * @brief Changes the direction of a specific GPIO pin.
   *
   * @param pin GPIOPin to change the direction for.
   * @param direction New direction for the pin.
   */
  void ChangePinDirection(const GPIOPin pin, const gpiod::line::direction direction) override;

  /**
   * @brief Returns true if a specific pin is configured and stored in GPIO info storage
   *
   * @param pin The GPIO pin to check availability for
   * @return true if the pin is available, false otherwise
   */
  bool IsPinAvailable(const GPIOPin pin) const override;

  /**
   * @brief Checks if a specific GPIO pin is active. This method returns the value stored in the
   * class read during the last edge event.
   *
   * @param pin GPIOPin to check.
   *
   * @throws std::runtime_error if GPIO monitor thread is not running.
   *
   * @return True if the pin is active, false otherwise.
   */
  bool IsPinActive(const GPIOPin pin) override;

  /**
   * @brief Sets the value for a specific GPIO pin.
   *
   * @param pin GPIOPin to set the value for.
   * @param value The boolean value to set for the pin.
   * @param pin_validation_wait_time The time duration to wait for the pin value to change before
   * checking if change was successful.
   *
   * @throws std::invalid_argument if trying to set the value for an INPUT pin.
   * @throws std::runtime_error if changing the GPIO state fails.
   *
   * @return true if the pin value is successfully set, false otherwise.
   */
  bool SetPinValue(
    const GPIOPin pin, const bool value,
    const std::chrono::milliseconds & pin_validation_wait_time =
      std::chrono::milliseconds(0)) override;

private:
  std::unique_ptr<gpiod::line_request> CreateLineRequest(gpiod::chip & chip);
  gpiod::line_settings GenerateLineSettings(const GPIOInfo & pin_info);
  GPIOPin GetPinFromOffset(const gpiod::line::offset & offset) const;
  GPIOInfo & GetGPIOInfoRef(const GPIOPin pin);
  void ConfigureLineRequest(
    gpiod::chip & chip, gpiod::request_builder & builder, GPIOInfo & gpio_info);
  void MonitorAsyncEvents();
  void HandleEdgeEvent(const gpiod::edge_event & event);
  bool IsGPIOMonitorThreadRunning() const;

  /**
   * @brief Enables asynchronous monitoring of GPIO pin events.
   *
   * @throws std::runtime_error if a timeout occurs while waiting for the GPIO monitor thread.
   */
  void GPIOMonitorOn();

  /**
   * @brief Disables asynchronous monitoring of GPIO pin events.
   */
  void GPIOMonitorOff();

  /**
   * @brief Callback function for GPIO edge events.
   *
   * @param gpio_info Information related to the state of the GPIO pin for which the event took
   * place.
   */
  std::function<void(const GPIOInfo & gpio_info)> GPIOEdgeEventCallback;

  /**
   * @brief Vector containing GPIO pin configuration information such as pin direction, value, etc.
   */
  std::vector<GPIOInfo> gpio_info_storage_;

  /**
   * @brief Mutex for managing access to GPIO pin information.
   */
  mutable std::shared_mutex gpio_info_storage_mutex_;

  /**
   * @brief Request object for controlling GPIO lines.
   */
  std::unique_ptr<gpiod::line_request> line_request_;

  /**
   * @brief Thread object for monitoring GPIO events asynchronously.
   */
  std::unique_ptr<std::thread> gpio_monitor_thread_;

  /**
   * @brief When enabled, the GPIODriver object will initiate the GPIO monitor
   * thread with real-time scheduling if a valid priority is provided.
   */
  bool use_rt_;

  /**
   * @brief Priority for the GPIO monitoring thread. The value ranges from 0 to 99, inclusive,
   * and is used to configure the FIFO real-time scheduling policy for the monitor thread
   * if real-time monitoring is enabled.
   */
  unsigned gpio_monit_thread_sched_priority_;
  std::atomic_bool gpio_monitor_thread_enabled_ = false;
  std::condition_variable monitor_init_cond_var_;
  std::mutex monitor_init_mtx_;
  static constexpr unsigned gpio_debounce_period_ = 10;
  static constexpr unsigned edge_event_buffer_size_ = 2;
  const std::filesystem::path gpio_chip_path_ = "/dev/gpiochip0";
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_GPIO_DRIVER_HPP_
