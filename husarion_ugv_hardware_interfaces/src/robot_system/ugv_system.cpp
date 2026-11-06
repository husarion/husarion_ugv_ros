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

#include "husarion_ugv_hardware_interfaces/robot_system/ugv_system.hpp"

#include <sys/sysinfo.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/logging.hpp>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/panther_robot_driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/system_ros_interface.hpp"
#include "husarion_ugv_hardware_interfaces/utils.hpp"

#include "husarion_ugv_utils/common_utilities.hpp"
#include "husarion_ugv_utils/diagnostics.hpp"

namespace husarion_ugv_hardware_interfaces
{

CallbackReturn UGVSystem::on_init(const hardware_interface::HardwareInfo & hardware_info)
{
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  try {
    CheckJointSize();
    SortAndCheckJointNames();
    SetInitialValues();
    CheckInterfaces();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "An exception occurred while initializing: " << e.what());
    return CallbackReturn::ERROR;
  }

  try {
    ReadDrivetrainSettings();
    ReadCANopenSettings();
    ReadRTThreadPriorities();
    ReadInitializationActivationAttempts();
    ReadUVPSettings();
    ReadParametersAndCreateRoboteqErrorFilter();
    ReadDriverStatesUpdateFrequency();
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "An exception occurred while reading the parameters: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_configure(const rclcpp_lifecycle::State &)
{
  try {
    ConfigureGPIOController();
    ConfigureRobotDriver();
    ConfigureEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to initialize GPIO, Motors, or E-Stop controllers. Error: " << e.what());
    return CallbackReturn::ERROR;
  }

  e_stop_diagnostics_ = e_stop_->GetDiagnostics();

  std::fill(hw_commands_velocities_.begin(), hw_commands_velocities_.end(), 0.0);
  std::fill(hw_states_positions_.begin(), hw_states_positions_.end(), 0.0);
  std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), 0.0);
  std::fill(hw_states_efforts_.begin(), hw_states_efforts_.end(), 0.0);

  if (!OperationWithAttempts(
        std::bind(&RobotDriverInterface::Activate, robot_driver_),
        max_roboteq_activation_attempts_)) {
    RCLCPP_ERROR_STREAM(
      logger_, "Failed to activate UGV System: Couldn't activate RobotDriver in "
                 << max_roboteq_activation_attempts_ << " attempts.");
    return CallbackReturn::ERROR;
  }

  system_ros_interface_ = std::make_unique<SystemROSInterface>("hardware_controller");
  io_state_diagnostics_ = system_ros_interface_->GetIOStateDiagnostics();

  // The services hold the GPIO controller weakly. A strong copy kept it alive past
  // gpio_controller_.reset() in the teardown, so its monitor thread outlived the ROS interface it
  // publishes through. Stopping the driver with the e-stop released then segfaulted: the latch
  // fires a GPIO edge inside that window.
  const std::weak_ptr<GPIOControllerInterface> weak_gpio_controller = gpio_controller_;
  const auto gpio_service =
    [weak_gpio_controller](bool (GPIOControllerInterface::*method)(const bool)) {
      return std::function<void(bool)>([weak_gpio_controller, method](const bool enable) {
        const auto gpio_controller = weak_gpio_controller.lock();
        if (!gpio_controller) {
          throw std::runtime_error("GPIO controller is shut down.");
        }
        ((*gpio_controller).*method)(enable);
      });
    };

  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/fan_enable", gpio_service(&GPIOControllerInterface::FanEnable));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/aux_power_enable", gpio_service(&GPIOControllerInterface::AUXPowerEnable));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/digital_power_enable", gpio_service(&GPIOControllerInterface::DigitalPowerEnable));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/charger_enable", gpio_service(&GPIOControllerInterface::ChargerEnable));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/led_control_enable", gpio_service(&GPIOControllerInterface::LEDControlEnable));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/motor_torque_enable",
    std::bind(&UGVSystem::MotorTorqueEnable, this, std::placeholders::_1));

  system_ros_interface_->AddService<TriggerSrv, std::function<void()>>(
    "hardware/e_stop_trigger", std::bind(&UGVSystem::TriggerEStopServiceCall, this), 1,
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto e_stop_reset_qos = rclcpp::ServicesQoS();
  e_stop_reset_qos.keep_last(1);
  system_ros_interface_->AddService<TriggerSrv, std::function<void()>>(
    "hardware/e_stop_reset", std::bind(&UGVSystem::ResetEStop, this), 2,
    rclcpp::CallbackGroupType::MutuallyExclusive, e_stop_reset_qos);

  system_ros_interface_->AddDiagnosticTask(
    std::string("System errors"), this, &UGVSystem::DiagnoseErrors);

  system_ros_interface_->AddDiagnosticTask(
    std::string("System status"), this, &UGVSystem::DiagnoseStatus);

  system_ros_interface_->AddDiagnosticTask(std::string("E-stop"), this, &UGVSystem::DiagnoseEStop);

  ConfigureUVP();

  gpio_controller_->RegisterGPIOEventCallback(
    [this](const auto & state) { system_ros_interface_->PublishIOState(state); });

  const auto io_state = gpio_controller_->QueryControlInterfaceIOStates();
  system_ros_interface_->InitializeAndPublishIOStateMsg(io_state);

  system_ros_interface_->PublishEStopStateMsg(e_stop_->ReadEStopState());

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  // Teardown order matters: the diagnostic updater inside
  // system_ros_interface_ keeps ticking on the executor and its tasks call
  // robot_driver_->GetData() - deinitializing the driver first leaves a
  // window where DiagnoseErrors() throws out of the timer callback and
  // std::terminate takes the whole node down (bit us live on a Lynx). The
  // GPIO event callback has the sibling problem: it publishes through
  // system_ros_interface_, so the GPIO controller has to go before the
  // interface it calls into. Stop callers before callees: GPIO watcher
  // first (e_stop_ holds refs to gpio + driver, drop it alongside), then
  // the ROS interface (stops the diagnostics executor), then the driver.
  e_stop_.reset();
  gpio_controller_.reset();

  system_ros_interface_.reset();

  robot_driver_->Deinitialize();
  robot_driver_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_activate(const rclcpp_lifecycle::State &)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    RCLCPP_INFO(logger_, "Triggering E-Stop due to deactivation.");
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Deactivation failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  try {
    RCLCPP_INFO(logger_, "Triggering E-Stop due to shutdown.");
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  // Same teardown order as on_cleanup - rationale there.
  e_stop_.reset();
  gpio_controller_.reset();

  system_ros_interface_.reset();

  robot_driver_->Deinitialize();
  robot_driver_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_error(const rclcpp_lifecycle::State &)
{
  try {
    RCLCPP_WARN(logger_, "Triggering E-Stop due to error.");
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Handling error failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  if (system_ros_interface_) {
    system_ros_interface_->BroadcastOnDiagnosticTasks(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "An error has occurred during a node state transition.");
  }

  // Same teardown order as on_cleanup - rationale there.
  e_stop_.reset();
  gpio_controller_.reset();

  system_ros_interface_.reset();

  robot_driver_->Deinitialize();
  robot_driver_.reset();

  return CallbackReturn::SUCCESS;
}

std::vector<StateInterface> UGVSystem::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces;
  for (std::size_t i = 0; i < joint_size_; i++) {
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_POSITION, &hw_states_positions_[i]));
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_states_velocities_[i]));
    state_interfaces.emplace_back(StateInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_EFFORT, &hw_states_efforts_[i]));
  }

  return state_interfaces;
}

std::vector<CommandInterface> UGVSystem::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces;
  for (std::size_t i = 0; i < joint_size_; i++) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      joints_names_sorted_[i], hardware_interface::HW_IF_VELOCITY, &hw_commands_velocities_[i]));
  }

  return command_interfaces;
}

return_type UGVSystem::read(const rclcpp::Time & time, const rclcpp::Duration & /* period */)
{
  timespec now_ts;
  clock_gettime(CLOCK_MONOTONIC, &now_ts);
  if (last_read_ts_.tv_sec != 0 || last_read_ts_.tv_nsec != 0) {
    read_cycle_ms_ = static_cast<float>(now_ts.tv_sec - last_read_ts_.tv_sec) * 1000.0F +
                     static_cast<float>(now_ts.tv_nsec - last_read_ts_.tv_nsec) / 1.0e6F;

    const auto read_cycle_us = static_cast<std::uint32_t>(read_cycle_ms_ * 1000.0F);
    auto read_cycle_max_us = read_cycle_max_us_.load(std::memory_order_relaxed);
    while (read_cycle_us > read_cycle_max_us &&
           !read_cycle_max_us_.compare_exchange_weak(
             read_cycle_max_us, read_cycle_us, std::memory_order_relaxed)) {
    }
  }
  last_read_ts_ = now_ts;

  UpdateMotorsState();

  if (time >= next_driver_state_update_time_) {
    UpdateDriverState();
    UpdateDriverStateMsg();
    system_ros_interface_->PublishRobotDriverState();
    // Step from the planned time so late reads do not stretch the period
    next_driver_state_update_time_ = next_driver_state_update_time_ + driver_states_update_period_;
    if (next_driver_state_update_time_ <= time) {
      next_driver_state_update_time_ = time + driver_states_update_period_;
    }
  }

  UpdateEStopState();

  return return_type::OK;
}

return_type UGVSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  const auto lifecycle_state = this->get_lifecycle_state().id();
  const bool e_stop = e_stop_->ReadEStopState();

  if (lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && !e_stop) {
    HandleRobotDriverWriteOperation([this] {
      const auto speed_cmds = GetSpeedCommands();
      robot_driver_->SendSpeedCommands(speed_cmds);
    });
  }

  return return_type::OK;
}

void UGVSystem::CheckJointSize() const
{
  if (info_.joints.size() != joint_size_) {
    throw std::runtime_error(
      "Wrong number of joints defined: " + std::to_string(info_.joints.size()) + ", " +
      std::to_string(joint_size_) + " expected.");
  }
}

void UGVSystem::SortAndCheckJointNames()
{
  // Sort joints names - later hw_states and hw_commands are accessed by static indexes, so it
  // is necessary to make sure that joints are in specific order and order of definitions in URDF
  // doesn't matter
  for (std::size_t i = 0; i < joint_size_; i++) {
    std::size_t match_count = 0;

    for (std::size_t j = 0; j < joint_size_; j++) {
      if (CheckIfJointNameContainValidSequence(info_.joints[j].name, joint_order_[i])) {
        joints_names_sorted_[i] = info_.joints[j].name;
        ++match_count;
      }
    }

    if (match_count != 1) {
      throw std::runtime_error(
        "There should be exactly one joint containing " + joint_order_[i] + ", " +
        std::to_string(match_count) + " found.");
    }
  }
}

void UGVSystem::SetInitialValues()
{
  // It isn't safe to set command to NaN - sometimes it could be interpreted as Inf (although it
  // shouldn't). In case of velocity, I think that setting the initial value to 0.0 is the best
  // option.
  hw_commands_velocities_.resize(joint_size_, 0.0);

  hw_states_positions_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
  hw_states_velocities_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
  hw_states_efforts_.resize(joint_size_, std::numeric_limits<double>::quiet_NaN());
}

void UGVSystem::CheckInterfaces() const
{
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    // Commands
    if (joint.command_interfaces.size() != 1) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + std::to_string(joint.command_interfaces.size()) +
        " command interfaces. 1 expected.");
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.command_interfaces[0].name +
        " command interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
    }

    // States
    if (joint.state_interfaces.size() != 3) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + std::to_string(joint.state_interfaces.size()) +
        " state  " + (joint.state_interfaces.size() == 1 ? "interface." : "interfaces.") +
        " 3 expected.");
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[0].name +
        " as first state interface. " + hardware_interface::HW_IF_POSITION + " expected.");
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[1].name +
        " as second state interface. " + hardware_interface::HW_IF_VELOCITY + " expected.");
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      throw std::runtime_error(
        "Joint " + joint.name + " has " + joint.state_interfaces[2].name +
        " as third state interface. " + hardware_interface::HW_IF_EFFORT + " expected.");
    }
  }
}

void UGVSystem::ReadDrivetrainSettings()
{
  drivetrain_settings_.motor_torque_constant =
    std::stof(info_.hardware_parameters["motor_torque_constant"]);
  drivetrain_settings_.gear_ratio = std::stof(info_.hardware_parameters["gear_ratio"]);
  drivetrain_settings_.gearbox_efficiency =
    std::stof(info_.hardware_parameters["gearbox_efficiency"]);
  drivetrain_settings_.encoder_resolution =
    std::stof(info_.hardware_parameters["encoder_resolution"]);
  drivetrain_settings_.max_rpm_motor_speed =
    std::stof(info_.hardware_parameters["max_rpm_motor_speed"]);
}

void UGVSystem::ReadCANopenSettings()
{
  canopen_settings_.can_interface_name = info_.hardware_parameters["can_interface_name"];
  canopen_settings_.master_can_id = std::stoi(info_.hardware_parameters["master_can_id"]);
  canopen_settings_.pdo_motor_states_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_motor_states_timeout_ms"]));
  canopen_settings_.pdo_driver_state_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["pdo_driver_state_timeout_ms"]));
  canopen_settings_.sdo_operation_timeout_ms =
    std::chrono::milliseconds(std::stoi(info_.hardware_parameters["sdo_operation_timeout_ms"]));

  ReadCANopenSettingsDriverCANIDs();
}

void UGVSystem::ReadRTThreadPriorities()
{
  canopen_settings_.canopen_thread_sched_priority = ReadRTPriority(
    "canopen_thread_priority", CANopenSettings().canopen_thread_sched_priority);
  canopen_settings_.rpdo_dispatch_sched_priority = ReadRTPriority(
    "rpdo_dispatch_priority", CANopenSettings().rpdo_dispatch_sched_priority);
  gpio_watchdog_sched_priority_ = ReadRTPriority(
    "gpio_watchdog_priority", Watchdog::kWatchdogSchedPriority);
  gpio_monitor_sched_priority_ = ReadRTPriority(
    "gpio_monitor_priority", GPIOController::kMonitorSchedPriority);

  canopen_settings_.canopen_thread_cpu = ReadRTCPU("canopen_thread_cpu", kDefaultCANCPU);
  canopen_settings_.rpdo_dispatch_cpu = ReadRTCPU("rpdo_dispatch_cpu", kDefaultCANCPU);
  gpio_watchdog_cpu_ = ReadRTCPU("gpio_watchdog_cpu", kDefaultControlCPU);
  gpio_monitor_cpu_ = ReadRTCPU("gpio_monitor_cpu", kDefaultControlCPU);
}

unsigned UGVSystem::ReadRTPriority(const std::string & name, const unsigned default_priority)
{
  // Optional, so a robot description written before these existed keeps the defaults.
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_priority;
  }

  const auto priority = ParseRTPriority(it->second);
  if (!priority) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid " << name << " '" << it->second << "', expected a whole number in ["
                          << kMinRTPriority << ", " << kMaxRTPriority << "]. Using the default "
                          << default_priority << ".");
    return default_priority;
  }
  return *priority;
}

int UGVSystem::ReadRTCPU(const std::string & name, const int default_cpu)
{
  // The default is checked too: a machine with fewer cores than the robot's computer gets an
  // unpinned thread and a warning instead of a failed pin.
  const auto it = info_.hardware_parameters.find(name);
  const std::string text = it == info_.hardware_parameters.end() ? std::to_string(default_cpu)
                                                                 : it->second;
  const int cpu_count = get_nprocs_conf();

  const auto cpu = ParseCPU(text, cpu_count);
  if (!cpu) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid " << name << " '" << text << "', expected -1 or a CPU in [0, "
                          << cpu_count - 1 << "]. The thread will not be pinned.");
    return -1;
  }
  return *cpu;
}

void UGVSystem::ReadInitializationActivationAttempts()
{
  max_roboteq_initialization_attempts_ =
    std::stoi(info_.hardware_parameters["max_roboteq_initialization_attempts"]);
  max_roboteq_activation_attempts_ =
    std::stoi(info_.hardware_parameters["max_roboteq_activation_attempts"]);
}

void UGVSystem::ReadParametersAndCreateRoboteqErrorFilter()
{
  const unsigned max_write_pdo_cmds_errors_count =
    std::stoi(info_.hardware_parameters["max_write_pdo_cmds_errors_count"]);
  const unsigned max_read_pdo_motor_states_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_motor_states_errors_count"]);
  const unsigned max_read_pdo_driver_state_errors_count =
    std::stoi(info_.hardware_parameters["max_read_pdo_driver_state_errors_count"]);

  roboteq_error_filter_ = std::make_shared<RoboteqErrorFilter>(
    max_write_pdo_cmds_errors_count, max_read_pdo_motor_states_errors_count,
    max_read_pdo_driver_state_errors_count, 1);
}

void UGVSystem::ReadDriverStatesUpdateFrequency()
{
  const float driver_states_update_frequency =
    std::stof(info_.hardware_parameters["driver_states_update_frequency"]);
  driver_states_update_period_ =
    rclcpp::Duration::from_seconds(1.0f / driver_states_update_frequency);
}

void UGVSystem::ConfigureGPIOController()
{
  gpio_controller_ = GPIOControllerFactory::CreateGPIOController(
    gpio_watchdog_sched_priority_, gpio_monitor_sched_priority_, gpio_watchdog_cpu_,
    gpio_monitor_cpu_);
  gpio_controller_->Start();

  RCLCPP_INFO(logger_, "Successfully configured GPIO controller.");
}

void UGVSystem::ConfigureRobotDriver()
{
  robot_driver_write_mtx_ = std::make_shared<std::mutex>();
  DefineRobotDriver();

  if (!OperationWithAttempts(
        std::bind(&RobotDriverInterface::Initialize, robot_driver_),
        max_roboteq_initialization_attempts_,
        std::bind(&RobotDriverInterface::Deinitialize, robot_driver_))) {
    throw std::runtime_error("Roboteq drivers initialization failed.");
  }

  RCLCPP_INFO(logger_, "Successfully configured robot driver");
}

void UGVSystem::ConfigureEStop()
{
  if (!gpio_controller_ || !roboteq_error_filter_ || !robot_driver_ || !robot_driver_write_mtx_) {
    throw std::runtime_error("Failed to configure E-Stop, make sure to setup entities first.");
  }

  e_stop_ = std::make_shared<EStop>(
    gpio_controller_, roboteq_error_filter_, robot_driver_, robot_driver_write_mtx_,
    std::bind(&UGVSystem::AreVelocityCommandsNearZero, this));

  RCLCPP_INFO(logger_, "Successfully configured E-Stop");
}

void UGVSystem::TriggerEStopServiceCall()
{
  RCLCPP_INFO(logger_, "Triggering E-Stop due to service call.");
  e_stop_->TriggerEStop();
}

void UGVSystem::ResetEStop()
{
  const auto lifecycle_state = this->get_lifecycle_state().id();

  if (lifecycle_state != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    throw std::runtime_error(
      "Can't reset E-Stop when the hardware interface is not in ACTIVE state.");
  }

  RCLCPP_INFO(logger_, "Resetting E-Stop.");

  e_stop_->ResetEStop();
  // The reset turns the Roboteq emergency stop off, which enables the motor torque again.
  torque_enable_pending_ = false;
  torque_disabled_by_request_ = false;
}

void UGVSystem::UpdateTorqueEnablePending(const bool emergency_stop_set)
{
  // The fault flags arrive in a TPDO up to 50 ms after the torque enable was written, and one
  // stale sample latches the error filter. The request ends only when no driver reports the
  // emergency stop any more, and the latched errors are cleared on that same sample (bit us live
  // on a Panther: one of its two drivers was always late and every torque enable read as a fault).
  if (torque_enable_pending_ && !emergency_stop_set) {
    torque_enable_pending_ = false;
    torque_disabled_by_request_ = false;
    roboteq_error_filter_->SetClearErrorsFlag();
  }
}

void UGVSystem::UpdateMotorsState()
{
  try {
    robot_driver_->UpdateMotorsState();
    UpdateHwStates();
    UpdateCANopenResyncWatchdog(UpdateMotorsStateDataTimedOut());
  } catch (const std::runtime_error & e) {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, steady_clock_, 5000,
      "An exception occurred while updating motors states: " << e.what());

    // A read that throws (heartbeat timeout, CAN error) is a timed-out read -
    // without this feed the watchdog is unreachable in exactly the state it
    // exists for, because a dead master's own symptom raises before the
    // success-path feed above (bit us live: a wedged master latched the
    // e-stop every 5 s for hours while the wire carried healthy heartbeats).
    UpdateCANopenResyncWatchdog(true);
  }
}

void UGVSystem::UpdateDriverState()
{
  try {
    robot_driver_->UpdateDriversState();
    UpdateFlagErrors();
    UpdateDriverStateDataTimedOut();
  } catch (const std::runtime_error & e) {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);

    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, steady_clock_, 5000,
      "An exception occurred while updating drivers states: " << e.what());
  }
}

void UGVSystem::UpdateCANopenResyncWatchdog(const bool data_timed_out)
{
  if (!data_timed_out) {
    pdo_timeout_ongoing_ = false;
    return;
  }

  const auto now = std::chrono::steady_clock::now();

  if (!pdo_timeout_ongoing_) {
    pdo_timeout_ongoing_ = true;
    pdo_timeout_since_ = now;
    pdo_timeout_events_.fetch_add(1, std::memory_order_relaxed);
    return;
  }

  if (now - pdo_timeout_since_ < pdo_timeout_resync_threshold_) {
    return;
  }

  // The master can come up (or be recreated into) a state where it no longer receives PDOs
  // while the bus itself stays healthy. It never recovers on its own and the process never
  // exits, so the container restart policy can't heal it. Exiting here reuses the restart
  // machinery for a clean re-initialization - simpler and more reliable than tearing down and
  // re-creating the master in-process.
  RCLCPP_FATAL_STREAM(
    logger_, "CANopen resync watchdog: " << pdo_timeout_resync_threshold_.count()
                                         << " s of continuous PDO timeout - restarting.");
  std::fflush(stdout);
  std::fflush(stderr);
  std::_Exit(resync_watchdog_exit_code_);
}

void UGVSystem::UpdateEStopState()
{
  if (robot_driver_->CommunicationError()) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 5000, "Triggering E-Stop due to robot driver communication error.");
    e_stop_->TriggerEStop();
  }

  const bool e_stop = e_stop_->ReadEStopState();
  if (uvp_guard_) {
    UpdateUVPGuard(e_stop);
  }
  system_ros_interface_->PublishEStopStateIfChanged(e_stop);
}

void UGVSystem::ReadUVPSettings()
{
  // Optional and absent from the robot description unless the protection is enabled, so the
  // description stays what it was without it.
  const auto it = info_.hardware_parameters.find("uvp_enable");
  uvp_enable_ = it != info_.hardware_parameters.end() &&
                (it->second == "true" || it->second == "True" || it->second == "1");
}

void UGVSystem::ConfigureUVP()
{
  if (!uvp_enable_) {
    return;
  }

  uvp_guard_ = std::make_unique<UvpEStopGuard>();
  uvp_configured_ns_ = EStopDiagnostics::SteadyNowNs();

  // A callback group of its own, so a busy e-stop service never delays the protection.
  system_ros_interface_->AddSubscription<BatteryProtectionMsg>(
    "battery/protection", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
    std::bind(&UGVSystem::ProtectionCB, this, std::placeholders::_1), 3);

  RCLCPP_INFO(logger_, "Battery undervoltage protection enabled.");
}

void UGVSystem::ProtectionCB(const std::shared_ptr<const BatteryProtectionMsg> msg)
{
  UvpProtectionInput input;
  input.received = true;
  input.state = msg->state;
  input.mode = msg->mode;
  input.reactions_enabled = msg->reactions_enabled;
  input.drive_window_s = msg->drive_window_s;
  input.trip_id = msg->trip_id;
  input.trip_stamp_ns = rclcpp::Time(msg->trip_stamp).nanoseconds();
  uvp_input_.writeFromNonRT(input);
  uvp_last_msg_ns_.store(EStopDiagnostics::SteadyNowNs());
}

void UGVSystem::UpdateUVPGuard(const bool e_stop)
{
  const auto * input = uvp_input_.readFromRT();
  if (!input) {
    return;
  }

  // Only ever a trigger: releasing the e-stop stays with the reset service and its checks.
  uvp_guard_->Update(*input, e_stop, EStopDiagnostics::SteadyNowNs(), [this]() {
    try {
      RCLCPP_WARN(logger_, "Triggering E-Stop due to battery undervoltage protection.");
      e_stop_->TriggerEStopFromUVP();
      return true;
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM_THROTTLE(
        logger_, steady_clock_, 5000,
        "Battery undervoltage protection failed to trigger the E-Stop: " << e.what());
      return false;
    }
  });
}

void UGVSystem::HandleRobotDriverWriteOperation(std::function<void()> write_operation)
{
  try {
    {
      std::unique_lock<std::mutex> robot_driver_write_lck(
        *robot_driver_write_mtx_, std::defer_lock);
      if (!robot_driver_write_lck.try_lock()) {
        throw std::runtime_error(
          "Can't acquire mutex for writing commands - E-stop is being triggered.");
      }
      write_operation();
    }

    roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);
  } catch (const std::runtime_error & e) {
    RCLCPP_WARN_STREAM(logger_, "An exception occurred while writing commands: " << e.what());
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  }
}

bool UGVSystem::AreVelocityCommandsNearZero()
{
  for (const auto & cmd : hw_commands_velocities_) {
    if (std::abs(cmd) > std::numeric_limits<double>::epsilon()) {
      return false;
    }
  }
  return true;
}

void UGVSystem::DiagnoseEStop(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  if (!e_stop_diagnostics_) {
    status.summary(diagnostic_updater::DiagnosticStatusWrapper::STALE, "E-stop state unavailable.");
    return;
  }

  const auto snapshot = e_stop_diagnostics_->Get();
  const auto now_ns = EStopDiagnostics::SteadyNowNs();
  const auto seconds_ago = [now_ns](const std::int64_t time_ns) {
    return static_cast<double>(now_ns - time_ns) / 1.0e9;
  };

  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message;
  std::string source;

  // A latch the driver caused or started in is the robot parked, not a fault. Only a stop by a
  // device outside the driver is worth the operator's attention.
  switch (snapshot.source) {
    case EStopLatchSource::RELEASED:
      message = "E-stop released.";
      source = "none";
      break;
    case EStopLatchSource::STARTUP:
      message = "E-stop latched since the driver started.";
      source = "driver start";
      break;
    case EStopLatchSource::SOFTWARE:
      message = "E-stop latched by software.";
      source = "software (service call, communication error or driver shutdown)";
      break;
    case EStopLatchSource::HARDWARE:
      level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
      message = "E-stop latched by a device outside the driver.";
      source = "hardware line (e-stop button, Roboteq fault or safety board watchdog)";
      break;
    case EStopLatchSource::UVP:
      level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
      message = "E-stop latched by the battery undervoltage protection.";
      source = "UVP (battery undervoltage protection)";
      break;
  }

  status.add(
    "State", snapshot.source == EStopLatchSource::RELEASED ? std::string("released")
                                                           : std::string("latched"));
  status.add("Source", source);
  if (snapshot.last_change_ns != 0) {
    status.add("Last change (s ago)", seconds_ago(snapshot.last_change_ns));
  }
  if (!snapshot.last_refused_reset.empty()) {
    status.add("Last refused reset", snapshot.last_refused_reset);
    status.add("Last refused reset (s ago)", seconds_ago(snapshot.last_refused_reset_ns));
  }

  if (uvp_guard_) {
    DiagnoseUVP(status, level, message);
  }

  status.summary(level, message);
}

void UGVSystem::DiagnoseUVP(
  diagnostic_updater::DiagnosticStatusWrapper & status, unsigned char & level,
  std::string & message)
{
  const auto now_ns = EStopDiagnostics::SteadyNowNs();

  status.add(
    "UVP",
    uvp_guard_->Active() ? std::string("tripped, holding the e-stop") : std::string("not tripped"));
  const auto window_end_ns = uvp_guard_->GetWindowEndNs();
  if (window_end_ns != 0) {
    status.add(
      "UVP drive window remaining (s)",
      std::max(0.0, static_cast<double>(window_end_ns - now_ns) / 1.0e9));
  }
  status.add("UVP e-stop triggers", uvp_guard_->GetTriggerCount());

  const auto last_msg_ns = uvp_last_msg_ns_.load();
  if (last_msg_ns == 0) {
    status.add("UVP data", "not received yet");
    if (static_cast<double>(now_ns - uvp_configured_ns_) / 1.0e9 > kUVPTopicStartupGrace) {
      level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
      message = "No battery protection data on battery/protection, the protection cannot act.";
    }
    return;
  }

  const auto age = static_cast<double>(now_ns - last_msg_ns) / 1.0e9;
  status.add("UVP data age (s)", age);
  if (age > kUVPTopicStaleTimeout) {
    level = diagnostic_updater::DiagnosticStatusWrapper::ERROR;
    message = "Battery protection data is stale, the protection cannot act.";
  }
}

void UGVSystem::DiagnoseDriversStatus(
  diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & robot_name)
{
  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message = robot_name + " system status monitoring.";

  try {
    for (const auto & driver : GetDriversDiagnosticData()) {
      const auto & state = driver.data.GetDriverState();
      status.add(driver.name + " voltage (V)", state.GetVoltage());
      status.add(driver.name + " battery current (A)", state.GetCurrent());
      status.add(driver.name + " temperature (\u00B0C)", state.GetTemperature());
      status.add(driver.name + " heatsink temperature (\u00B0C)", state.GetHeatsinkTemperature());
      status.add(
        "Motor current " + driver.channel_1_motor + " (A)",
        driver.data.GetMotorState(RoboteqDriver::kChannel1).GetCurrent());
      status.add(
        "Motor current " + driver.channel_2_motor + " (A)",
        driver.data.GetMotorState(RoboteqDriver::kChannel2).GetCurrent());

      if (state.GetHeatsinkTemperature() >= kHeatsinkTemperatureWarn) {
        level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
        message = driver.name + " heatsink temperature is close to the overheat limit.";
      }
    }
  } catch (const std::exception & e) {
    status.summary(
      diagnostic_updater::DiagnosticStatusWrapper::STALE,
      "Hardware data unavailable (shutting down?): " + std::string(e.what()));
    return;
  }

  const auto now = std::chrono::steady_clock::now();
  const auto pdo_timeout_events = pdo_timeout_events_.load(std::memory_order_relaxed);
  pdo_timeout_history_.emplace_back(now, pdo_timeout_events - pdo_timeout_events_in_history_);
  pdo_timeout_events_in_history_ = pdo_timeout_events;
  while (!pdo_timeout_history_.empty() &&
         now - pdo_timeout_history_.front().first > std::chrono::hours(1)) {
    pdo_timeout_history_.pop_front();
  }
  std::uint32_t pdo_timeouts_last_hour = 0;
  for (const auto & [time, events] : pdo_timeout_history_) {
    pdo_timeouts_last_hour += events;
  }
  status.add("PDO timeouts in the last hour", pdo_timeouts_last_hour);

  const auto read_cycle_max_us = read_cycle_max_us_.exchange(0, std::memory_order_relaxed);
  if (read_cycle_max_us > 0) {
    status.add("Read cycle max since the last report (ms)", read_cycle_max_us / 1000.0);
  }

  const auto io_state = io_state_diagnostics_ ? io_state_diagnostics_->Get() : IOStateDiagnostics{};
  const auto add_io = [&status](const std::string & key, const std::optional<bool> & value) {
    if (value.has_value()) {
      status.add(key, value.value() ? std::string("on") : std::string("off"));
    }
  };
  add_io("Motor power", io_state.motor_on);
  add_io("Aux power", io_state.aux_power);
  add_io("Fan", io_state.fan);
  if (io_state.charger_connected.has_value()) {
    status.add(
      "Charger connected",
      io_state.charger_connected.value() ? std::string("yes") : std::string("no"));
  }

  status.summary(level, message);
}

void UGVSystem::DiagnoseDriversErrors(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message = "No error detected.";
  const auto raise = [&level, &message](const unsigned char new_level, const std::string & text) {
    if (new_level > level) {
      level = new_level;
      message = text;
    }
  };

  try {
    // A torque disable on request sets the Roboteq emergency stop fault, and the error filter
    // latches it. That is the robot doing what it was asked, so it is a WARN while it is the only
    // fault.
    const bool torque_disabled_by_request = torque_disabled_by_request_.load();

    // A latched e-stop holds the motors through the Roboteq safety stop, so on a parked robot
    // that flag is the expected state, not a warning.
    const bool e_stop_latched = e_stop_diagnostics_ &&
                                e_stop_diagnostics_->GetSource() != EStopLatchSource::RELEASED;
    bool flag_error_not_torque_disable = false;

    for (const auto & driver : GetDriversDiagnosticData()) {
      const auto & data = driver.data;
      const std::string error_prefix = driver.name + " error: ";

      // With the torque disabled on request the emergency stop fault is that request, also when
      // other faults are set next to it.
      const bool torque_disable_fault = torque_disabled_by_request &&
                                        data.GetFaultFlag().IsEmergencyStopSet();
      if (torque_disable_fault) {
        raise(
          diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor torque disabled on request.");
        status.add(driver.name + " warning: motor torque", "disabled on request");
      }
      auto flag_errors = data.GetFlagErrorMap();
      if (torque_disable_fault) {
        flag_errors.erase(FaultFlag::kEmergencyStopKey);
        // Bits without a name mean nothing to this driver. On lynx-165f a torque disable left a
        // flag error set with no named flag but the emergency stop, so one of them came with it.
        // While the torque is off on request they are shown, but not as a fault.
        for (auto it = flag_errors.begin(); it != flag_errors.end();) {
          if (it->second && FlagError::IsUnnamedBitKey(it->first)) {
            status.add(driver.name + " warning: " + it->first, true);
            it = flag_errors.erase(it);
          } else {
            ++it;
          }
        }
      }
      if (std::any_of(flag_errors.begin(), flag_errors.end(), [](const auto & flag) {
            return flag.second;
          })) {
        flag_error_not_torque_disable = true;
        raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Error detected.");
        husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, flag_errors, error_prefix);
      }
      if (data.IsCANError()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Error detected.");
        status.add(error_prefix + "can_error", true);
      }
      if (data.IsHeartbeatTimeout()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Error detected.");
        status.add(error_prefix + "heartbeat_timeout", true);
      }

      const std::string warning_prefix = driver.name + " warning: ";
      const auto add_motor_warning = [&](const std::uint8_t channel, const std::string & motor) {
        const auto & runtime_error = data.GetRuntimeError(channel);
        const auto description = runtime_error.GetWarningDescription(!e_stop_latched);
        if (!description.empty()) {
          raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor " + description + ".");
          status.add(warning_prefix + motor + " motor", description);
        }
        if (e_stop_latched && runtime_error.IsSafetyStopActive()) {
          status.add(driver.name + ": " + motor + " motor", "safety stop (held by the e-stop)");
        }
      };
      add_motor_warning(RoboteqDriver::kChannel1, driver.channel_1_motor);
      add_motor_warning(RoboteqDriver::kChannel2, driver.channel_2_motor);

      if (data.IsMotorStatesDataTimedOut()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data is late.");
        status.add(warning_prefix + "motor_states_data_timed_out", true);
      }
      if (data.IsDriverStateDataTimedOut()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data is late.");
        status.add(warning_prefix + "driver_state_data_timed_out", true);
      }
    }

    const auto pdo_timeout_events = pdo_timeout_events_.load(std::memory_order_relaxed);
    const auto new_pdo_timeout_events = pdo_timeout_events - pdo_timeout_events_reported_;
    pdo_timeout_events_reported_ = pdo_timeout_events;
    if (new_pdo_timeout_events > 0) {
      raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data was late.");
      status.add("PDO timeouts since the last report", new_pdo_timeout_events);
    }

    // The filter latches: these stay set until the E-stop is reset or the motor torque toggled,
    // even after the cause went away.
    const bool only_torque_disable_latched =
      torque_disabled_by_request && !flag_error_not_torque_disable &&
      roboteq_error_filter_->IsError(ErrorsFilterIds::ROBOTEQ_DRIVER) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);
    if (roboteq_error_filter_->IsError() && !only_torque_disable_latched) {
      raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, "Error detected.");
      husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(
        status, roboteq_error_filter_->GetErrorDescriptionMap());
      status.add("Latched errors clear on", "E-stop reset or motor torque toggle");
    }
  } catch (const std::exception & e) {
    status.summary(
      diagnostic_updater::DiagnosticStatusWrapper::STALE,
      "Hardware data unavailable (shutting down?): " + std::string(e.what()));
    return;
  }

  status.summary(level, message);
}

void UGVSystem::MotorTorqueEnable(const bool enable)
{
  const bool e_stop = e_stop_->ReadEStopState();
  if (!e_stop) {
    throw std::runtime_error("Can't enable/disable torque when E-Stop is not triggered.");
  }

  try {
    {
      std::lock_guard<std::mutex> lck_g(*robot_driver_write_mtx_);

      // Marked before the write: an SDO write that times out may still have reached the Roboteq,
      // and the diagnostics must not read the torque disable it caused as a fault.
      if (!enable) {
        torque_enable_pending_ = false;
        torque_disabled_by_request_ = true;
        robot_driver_->TurnOnEStop();
      } else {
        robot_driver_->TurnOffEStop();
        torque_enable_pending_ = true;
      }
    }

    if (!enable) {
      roboteq_error_filter_->SetClearErrorsFlag();
    }
  } catch (const std::runtime_error & e) {
    auto warn = "An exception occurred while changing the torque state of the motors: " +
                std::string(e.what());
    RCLCPP_WARN_STREAM(logger_, warn);
    throw std::runtime_error(warn);
  }
}

}  // namespace husarion_ugv_hardware_interfaces
