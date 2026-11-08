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

#include <pthread.h>
#include <unistd.h>

#include <cerrno>
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include "husarion_ugv_hardware_interfaces/utils.hpp"
#include "husarion_ugv_utils/configure_rt.hpp"

namespace husarion_ugv_hardware_interfaces
{

namespace
{

// Writes dcf_dir/master.dcf, with every relative UploadFile path made absolute against dcf_dir, to
// a new private temporary file and returns its path.
std::filesystem::path WriteDCFWithAbsoluteUploadFiles(const std::filesystem::path & dcf_dir)
{
  const auto source_path = dcf_dir / "master.dcf";
  std::ifstream source(source_path);
  if (!source) {
    throw std::runtime_error("Can't read " + source_path.string());
  }
  std::ostringstream dcf;
  dcf << source.rdbuf();
  const std::string content = MakeDCFUploadFilesAbsolute(dcf.str(), dcf_dir);

  // A filesystem_error is a system_error, which Initialize() handles as a CAN error and returns
  // without a master, so it leaves as a runtime_error.
  std::filesystem::path temp_dir;
  try {
    temp_dir = std::filesystem::temp_directory_path();
  } catch (const std::filesystem::filesystem_error & e) {
    throw std::runtime_error(
      "Can't find a directory for the temporary master dcf: " + std::string(e.what()));
  }

  // mkstemps creates the file with O_EXCL and mode 0600 under a random name.
  std::string copy_path = (temp_dir / "husarion_ugv_master_XXXXXX.dcf").string();
  const int fd = mkstemps(copy_path.data(), 4);
  if (fd < 0) {
    throw std::runtime_error(
      "Can't create a temporary master dcf " + copy_path + ": " + std::strerror(errno));
  }
  const auto fail = [&copy_path](const int fd_to_close, const int error) {
    if (fd_to_close >= 0) {
      close(fd_to_close);
    }
    unlink(copy_path.c_str());
    throw std::runtime_error("Can't write " + copy_path + ": " + std::strerror(error));
  };
  std::size_t written = 0;
  while (written < content.size()) {
    const auto n = write(fd, content.data() + written, content.size() - written);
    if (n < 0 && errno == EINTR) {
      continue;
    }
    if (n <= 0) {
      fail(fd, n < 0 ? errno : EIO);
    }
    written += static_cast<std::size_t>(n);
  }
  if (close(fd) != 0) {
    fail(-1, errno);
  }
  return copy_path;
}

// Removes the file when it goes out of scope, also when the constructor that reads it throws.
struct ScopedFileRemover
{
  std::filesystem::path path;
  ~ScopedFileRemover()
  {
    if (!path.empty()) {
      std::error_code ec;
      std::filesystem::remove(path, ec);
    }
  }
};

}  // namespace

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

  // The calling thread, and every thread it creates afterwards, inherits this priority. With 0 it
  // keeps the scheduling it has.
  try {
    husarion_ugv_utils::ConfigureRT(canopen_settings_.configure_thread_sched_priority);
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
    pthread_setname_np(pthread_self(), "lely-io");
    try {
      husarion_ugv_utils::ConfigureRT(
        canopen_settings_.canopen_thread_sched_priority, canopen_settings_.canopen_thread_cpu);
    } catch (const std::runtime_error & e) {
      std::cerr << "Failed to configure RT priority for the CANopen thread: " << e.what()
                << std::endl
                << "Continuing with regular thread settings (it may have a negative impact on "
                   "performance)."
                << std::endl;
    }

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
  auto config_dir = std::filesystem::path(ament_index_cpp::get_package_share_directory(
                      "husarion_ugv_hardware_interfaces")) /
                    "config";
  std::string master_dcf_path = config_dir / "master.dcf";

  // The limited master.dcf names the slave configurations as UploadFile=slave_N.bin, and lely
  // opens that path as written on every slave boot. A copy with absolute paths keeps that
  // independent of the working directory. lely parses it in the AsyncMaster constructor.
  ScopedFileRemover absolute_dcf;
  if (canopen_settings_.limit_tpdo_rates) {
    absolute_dcf.path = WriteDCFWithAbsoluteUploadFiles(config_dir / "limited_tpdo_rates");
    master_dcf_path = absolute_dcf.path;
  }

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
