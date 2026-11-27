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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"

#include <condition_variable>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <system_error>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "husarion_ugv_utils/configure_rt.hpp"

namespace husarion_ugv_hardware_interfaces
{

CANopenManager::CANopenManager(const CANopenSettings & canopen_settings)
: canopen_settings_(canopen_settings)
{
}

void CANopenManager::Initialize()
{
  if (initialized_) {
    return;
  }

  canopen_communication_started_.store(false);

  try {
    husarion_ugv_utils::ConfigureRT(kCANopenThreadSchedPriority);
  } catch (const std::runtime_error & e) {
    std::cerr << "An exception occurred while configuring RT: " << e.what() << std::endl
              << "Continuing with regular thread settings (it may have a negative impact on the "
                 "performance)."
              << std::endl;
  }

  try {
    InitializeCANCommunication();
  } catch (const std::system_error & e) {
    std::cerr << "An exception occurred while initializing CAN: " << e.what() << std::endl;
    NotifyCANCommunicationStarted(false);
    return;
  }

  initialized_ = true;
}

void CANopenManager::StopCommunication()
{
  if (canopen_communication_thread_.joinable()) {
    if (master_) {
      master_->AsyncDeconfig().submit(*exec_, [this]() { ctx_->shutdown(); });
    }
    // Unbounded: deconfiguration completes after every registered driver has deconfigured.
    canopen_communication_thread_.join();
  }

  canopen_communication_started_.store(false);
}

void CANopenManager::Activate()
{
  if (!initialized_) {
    throw std::runtime_error("CANopenManager not initialized.");
  }

  canopen_communication_thread_ = std::thread([this]() {
    NotifyCANCommunicationStarted(true);

    // run(ec) reports the last failed operation's error even when the loop was stopped normally, so
    // report it only when it was not. A loop that stops with an error makes SDO and PDO operations
    // time out, and the system goes to the error state.
    std::error_code ec;
    loop_->run(ec);
    if (ec && !loop_->stopped()) {
      std::cerr << "CANopen event loop stopped with error: " << ec.message() << std::endl;
    }
  });

  // The thread can notify before this one starts waiting, and on failure the flag never becomes
  // true, so wait for the flag with a deadline.
  {
    std::unique_lock<std::mutex> lck(canopen_communication_started_mtx_);
    canopen_communication_started_cond_.wait_for(lck, kCANopenCommunicationStartTimeout, [this]() {
      return canopen_communication_started_.load();
    });
  }

  if (!canopen_communication_started_.load()) {
    throw std::runtime_error("CAN communication not activated.");
  }
}

void CANopenManager::Deinitialize()
{
  // Deinitialization should be done regardless of the initialized_ state - in case some operation
  // during initialization fails, it is still necessary to do the cleanup

  StopCommunication();

  master_.reset();
  chan_.reset();
  ctrl_.reset();
  timer_.reset();
  exec_.reset();
  loop_.reset();
  poll_.reset();
  ctx_.reset();

  initialized_ = false;
}

void CANopenManager::InitializeCANCommunication()
{
  lely::io::IoGuard io_guard;

  ctx_ = std::make_shared<lely::io::Context>();
  poll_ = std::make_shared<lely::io::Poll>(*ctx_);
  loop_ = std::make_shared<lely::ev::Loop>(poll_->get_poll());
  exec_ = std::make_shared<lely::ev::Executor>(loop_->get_executor());

  timer_ = std::make_shared<lely::io::Timer>(*poll_, *exec_, CLOCK_MONOTONIC);

  ctrl_ = std::make_shared<lely::io::CanController>(canopen_settings_.can_interface_name.c_str());
  chan_ = std::make_shared<lely::io::CanChannel>(*poll_, *exec_);

  chan_->open(*ctrl_);

  // Master dcf is generated from roboteq_motor_controllers_v80_21a using following command:
  // dcfgen canopen_configuration.yaml -r
  // dcfgen comes with lely, -r option tells to enable remote PDO mapping
  std::string master_dcf_path = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                                  "husarion_ugv_hardware_interfaces")) /
                                "config" / "master.dcf";

  master_ = std::make_shared<lely::canopen::AsyncMaster>(
    *timer_, *chan_, master_dcf_path, "", canopen_settings_.master_can_id);

  // Start the NMT service of the master by pretending to receive a 'reset node' command.
  master_->Reset();
}

void CANopenManager::NotifyCANCommunicationStarted(const bool result)
{
  {
    std::lock_guard<std::mutex> lck_g(canopen_communication_started_mtx_);
    canopen_communication_started_.store(result);
  }
  canopen_communication_started_cond_.notify_all();
}

}  // namespace husarion_ugv_hardware_interfaces
