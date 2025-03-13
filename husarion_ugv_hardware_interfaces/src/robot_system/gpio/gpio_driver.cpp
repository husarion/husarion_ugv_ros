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

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_driver.hpp"

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "gpiod.hpp"

#include "husarion_ugv_utils/configure_rt.hpp"

namespace husarion_ugv_hardware_interfaces
{

GPIODriver::GPIODriver(std::vector<GPIOInfo> gpio_info_storage)
: gpio_info_storage_(std::move(gpio_info_storage))
{
  if (gpio_info_storage_.empty()) {
    throw std::runtime_error("The GPIO information vector is empty.");
  }

  auto gpio_chip = gpiod::chip(gpio_chip_path_);
  line_request_ = CreateLineRequest(gpio_chip);
}

GPIODriver::~GPIODriver()
{
  for (GPIOInfo & gpio_info : gpio_info_storage_) {
    if (gpio_info.direction == gpiod::line::direction::OUTPUT) {
      line_request_->set_value(gpio_info.offset, gpio_info.init_value);
    }
  }

  GPIOMonitorOff();
  line_request_->release();
}

void GPIODriver::GPIOMonitorEnable(
  const bool use_rt, const unsigned gpio_monit_thread_sched_priority)
{
  use_rt_ = use_rt;
  gpio_monit_thread_sched_priority_ = gpio_monit_thread_sched_priority;

  GPIOMonitorOn();
}

void GPIODriver::ConfigureEdgeEventCallback(const std::function<void(const GPIOInfo &)> & callback)
{
  if (!IsGPIOMonitorThreadRunning()) {
    throw std::runtime_error("GPIO monitor thread is not running!");
  }

  GPIOEdgeEventCallback = callback;
}

std::unique_ptr<gpiod::line_request> GPIODriver::CreateLineRequest(gpiod::chip & chip)
{
  auto request_builder = chip.prepare_request();
  request_builder.set_consumer("husarion_ugv_hardware_interfaces");

  for (GPIOInfo & gpio_info : gpio_info_storage_) {
    ConfigureLineRequest(chip, request_builder, gpio_info);
  }

  return std::make_unique<gpiod::line_request>(request_builder.do_request());
}

void GPIODriver::ConfigureLineRequest(
  gpiod::chip & chip, gpiod::request_builder & builder, GPIOInfo & gpio_info)
{
  gpiod::line_settings settings = GenerateLineSettings(gpio_info);

  std::string pin_name;
  try {
    pin_name = pin_names_.at(gpio_info.pin);
  } catch (const std::out_of_range & e) {
    throw std::runtime_error("No name defined for one of pins: " + std::string(e.what()));
  }

  gpiod::line::offset offset = chip.get_line_offset_from_name(pin_name);

  builder.add_line_settings(offset, settings);
  gpio_info.offset = offset;
}

gpiod::line_settings GPIODriver::GenerateLineSettings(const GPIOInfo & gpio_info)
{
  auto settings = gpiod::line_settings();
  settings.set_direction(gpio_info.direction);
  settings.set_active_low(gpio_info.active_low);

  // Set the initial value only when the line is configured for the first time;
  // otherwise, set the last known value
  gpiod::line::value new_output_value = line_request_ ? gpio_info.value : gpio_info.init_value;
  settings.set_output_value(new_output_value);

  if (gpio_info.direction == gpiod::line::direction::INPUT) {
    settings.set_edge_detection(gpiod::line::edge::BOTH);
    settings.set_debounce_period(std::chrono::milliseconds(gpio_debounce_period_));
  }

  return settings;
}

void GPIODriver::ChangePinDirection(const GPIOPin pin, const gpiod::line::direction direction)
{
  std::lock_guard lock(gpio_info_storage_mutex_);
  GPIOInfo & gpio_info = GetGPIOInfoRef(pin);

  if (gpio_info.direction == direction) {
    return;
  }

  gpio_info.direction = direction;

  auto line_config = gpiod::line_config();

  for (const auto & info : gpio_info_storage_) {
    gpiod::line_settings settings = GenerateLineSettings(info);
    line_config.add_line_settings(info.offset, settings);
  }

  line_request_->reconfigure_lines(line_config);
  gpio_info.value = line_request_->get_value(gpio_info.offset);
}

bool GPIODriver::IsPinAvailable(const GPIOPin pin) const
{
  return std::any_of(gpio_info_storage_.begin(), gpio_info_storage_.end(), [&](const auto & info) {
    return info.pin == pin;
  });
}

bool GPIODriver::IsPinActive(const GPIOPin pin)
{
  if (!IsGPIOMonitorThreadRunning()) {
    throw std::runtime_error("GPIO monitor thread is not running!");
  }

  std::lock_guard lock(gpio_info_storage_mutex_);
  const GPIOInfo & pin_info = GetGPIOInfoRef(pin);
  return pin_info.value == gpiod::line::value::ACTIVE;
}

bool GPIODriver::SetPinValue(
  const GPIOPin pin, const bool value, const std::chrono::milliseconds & pin_validation_wait_time)
{
  GPIOInfo & gpio_info = GetGPIOInfoRef(pin);

  if (gpio_info.direction == gpiod::line::direction::INPUT) {
    throw std::invalid_argument("Cannot set value for INPUT pin.");
  }

  gpiod::line::value gpio_value = value ? gpiod::line::value::ACTIVE : gpiod::line::value::INACTIVE;

  std::unique_lock lock(gpio_info_storage_mutex_);

  try {
    line_request_->set_value(gpio_info.offset, gpio_value);

    if (pin_validation_wait_time.count() > 0) {
      lock.unlock();
      std::this_thread::sleep_for(pin_validation_wait_time);
      lock.lock();
    }

    if (line_request_->get_value(gpio_info.offset) != gpio_value) {
      throw std::runtime_error("Failed to change GPIO state.");
    }

    gpio_info.value = gpio_value;
    if (GPIOEdgeEventCallback) {
      GPIOEdgeEventCallback(gpio_info);
    }

    return true;
  } catch (const std::exception & e) {
    std::cerr << "An exception occurred while setting GPIO pin value: " << e.what() << std::endl;
    return false;
  }
}

void GPIODriver::GPIOMonitorOn()
{
  if (IsGPIOMonitorThreadRunning()) {
    return;
  }

  {
    std::lock_guard lock(gpio_info_storage_mutex_);
    for (auto & info : gpio_info_storage_) {
      info.value = line_request_->get_value(info.offset);
    }
  }

  std::unique_lock<std::mutex> lck(monitor_init_mtx_);

  gpio_monitor_thread_enabled_ = true;
  gpio_monitor_thread_ = std::make_unique<std::thread>(&GPIODriver::MonitorAsyncEvents, this);

  if (
    monitor_init_cond_var_.wait_for(lck, std::chrono::milliseconds(100)) ==
    std::cv_status::timeout) {
    throw std::runtime_error("Timeout while waiting for GPIO monitor thread.");
  }
}

void GPIODriver::MonitorAsyncEvents()
{
  if (use_rt_) {
    husarion_ugv_utils::ConfigureRT(gpio_monit_thread_sched_priority_);
  }

  auto edge_event_buffer = gpiod::edge_event_buffer(edge_event_buffer_size_);

  {
    std::lock_guard<std::mutex> lck(monitor_init_mtx_);
  }
  monitor_init_cond_var_.notify_all();

  while (gpio_monitor_thread_enabled_) {
    if (line_request_->wait_edge_events(std::chrono::milliseconds(10))) {
      line_request_->read_edge_events(edge_event_buffer);

      for (const auto & event : edge_event_buffer) {
        HandleEdgeEvent(event);
      }
    }
  }
}

void GPIODriver::HandleEdgeEvent(const gpiod::edge_event & event)
{
  std::lock_guard lock(gpio_info_storage_mutex_);
  GPIOPin pin;

  try {
    pin = GetPinFromOffset(event.line_offset());
  } catch (const std::out_of_range & e) {
    std::cerr << "An exception occurred while handling edge event: " << e.what() << std::endl;
    return;
  }

  GPIOInfo & gpio_info = GetGPIOInfoRef(pin);
  gpiod::line::value new_value;

  if (event.type() == gpiod::edge_event::event_type::RISING_EDGE) {
    new_value = gpiod::line::value::ACTIVE;
  } else {
    new_value = gpiod::line::value::INACTIVE;
  }

  gpio_info.value = new_value;

  if (GPIOEdgeEventCallback) {
    GPIOEdgeEventCallback(gpio_info);
  }
}

void GPIODriver::GPIOMonitorOff()
{
  gpio_monitor_thread_enabled_ = false;

  if (IsGPIOMonitorThreadRunning()) {
    gpio_monitor_thread_->join();
  }
}

bool GPIODriver::IsGPIOMonitorThreadRunning() const
{
  return gpio_monitor_thread_ && gpio_monitor_thread_->joinable();
}

GPIOInfo & GPIODriver::GetGPIOInfoRef(const GPIOPin pin)
{
  for (auto & info : gpio_info_storage_) {
    if (info.pin == pin) {
      return info;
    }
  }

  throw std::invalid_argument("Pin not found in GPIO info storage.");
}

GPIOPin GPIODriver::GetPinFromOffset(const gpiod::line::offset & offset) const
{
  for (const auto & gpio_info : gpio_info_storage_) {
    if (gpio_info.offset == offset) {
      return gpio_info.pin;
    }
  }

  throw std::out_of_range(
    "Pin with offset " + std::to_string(offset) + " not found in GPIO info storage.");
}

}  // namespace husarion_ugv_hardware_interfaces
