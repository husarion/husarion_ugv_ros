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

  // The new drivers count their PDO timeouts from zero.
  diagnostics_state_.pdo_timeouts_reported = 0;
  diagnostics_state_.pdo_timeouts_in_history = 0;
  diagnostics_state_.pdo_timeout_history.clear();
  diagnostics_state_.read_period_max_us.store(0);
  diagnostics_state_.last_read_ts = {0, 0};
  diagnostics_state_.torque_request.store(TorqueRequest::NONE);

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

  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/fan_enable",
    std::bind(&GPIOControllerInterface::FanEnable, gpio_controller_, std::placeholders::_1));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/aux_power_enable",
    std::bind(&GPIOControllerInterface::AUXPowerEnable, gpio_controller_, std::placeholders::_1));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/digital_power_enable",
    std::bind(
      &GPIOControllerInterface::DigitalPowerEnable, gpio_controller_, std::placeholders::_1));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/charger_enable",
    std::bind(&GPIOControllerInterface::ChargerEnable, gpio_controller_, std::placeholders::_1));
  system_ros_interface_->AddService<SetBoolSrv, std::function<void(bool)>>(
    "hardware/led_control_enable",
    std::bind(&GPIOControllerInterface::LEDControlEnable, gpio_controller_, std::placeholders::_1));
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

  // The callback runs on the GPIO monitor thread and on the thread of any SetPinValue() caller,
  // also while the teardown resets the ROS interface. It publishes under a mutex the teardown
  // takes to clear the pointer, so a call in flight finishes before the interface goes.
  {
    std::lock_guard<std::mutex> lck(ros_interface_for_callbacks_mtx_);
    ros_interface_for_callbacks_ = system_ros_interface_.get();
  }
  gpio_controller_->RegisterGPIOEventCallback([this](const auto & state) {
    std::lock_guard<std::mutex> lck(ros_interface_for_callbacks_mtx_);
    if (ros_interface_for_callbacks_) {
      ros_interface_for_callbacks_->PublishIOState(state);
    }
  });

  const auto io_state = gpio_controller_->QueryControlInterfaceIOStates();
  system_ros_interface_->InitializeAndPublishIOStateMsg(io_state);

  system_ros_interface_->PublishEStopStateMsg(e_stop_->ReadEStopState());

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  DestroyComponents();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_activate(const rclcpp_lifecycle::State &)
{
  // read() runs while inactive too, but a deactivate and activate in between is not a period.
  diagnostics_state_.restart_read_period.store(true);

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
  // e_stop_ is null when configure failed before it was created. A failed trigger still tears
  // down, which latches the e-stop through the watchdog anyway.
  bool trigger_failed = false;
  if (e_stop_) {
    try {
      RCLCPP_INFO(logger_, "Triggering E-Stop due to shutdown.");
      e_stop_->TriggerEStop();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Shutdown failed: " << e.what());
      trigger_failed = true;
    }
  }

  DestroyComponents();

  return trigger_failed ? CallbackReturn::ERROR : CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_error(const rclcpp_lifecycle::State &)
{
  // e_stop_ is null when configure failed before it was created. A failed trigger still tears
  // down, which latches the e-stop through the watchdog anyway.
  bool trigger_failed = false;
  if (e_stop_) {
    try {
      RCLCPP_WARN(logger_, "Triggering E-Stop due to error.");
      e_stop_->TriggerEStop();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Handling error failed: " << e.what());
      trigger_failed = true;
    }
  }

  if (system_ros_interface_) {
    system_ros_interface_->BroadcastOnDiagnosticTasks(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "An error has occurred during a node state transition.");
  }

  DestroyComponents();

  return trigger_failed ? CallbackReturn::ERROR : CallbackReturn::SUCCESS;
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
  UpdateReadCycleMax();

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
  canopen_settings_.configure_thread_sched_priority = ReadRTPriority(
    "configure_thread_priority", CANopenSettings().configure_thread_sched_priority);
  // These two must run above the control loop, so 0 is rejected for them.
  canopen_settings_.canopen_thread_sched_priority = ReadRTPriority(
    "canopen_thread_priority", CANopenSettings().canopen_thread_sched_priority, true);
  canopen_settings_.rpdo_dispatch_sched_priority = ReadRTPriority(
    "rpdo_dispatch_priority", CANopenSettings().rpdo_dispatch_sched_priority);
  gpio_watchdog_sched_priority_ = ReadRTPriority(
    "gpio_watchdog_priority", Watchdog::kWatchdogSchedPriority, true);
  gpio_monitor_sched_priority_ = ReadRTPriority(
    "gpio_monitor_priority", GPIOController::kMonitorSchedPriority);

  canopen_settings_.canopen_thread_cpu = ReadRTCPU("canopen_thread_cpu", kDefaultRTCPU);
  canopen_settings_.rpdo_dispatch_cpu = ReadRTCPU("rpdo_dispatch_cpu", kDefaultRTCPU);
  gpio_watchdog_cpu_ = ReadRTCPU("gpio_watchdog_cpu", kDefaultRTCPU);
  gpio_monitor_cpu_ = ReadRTCPU("gpio_monitor_cpu", kDefaultRTCPU);
}

unsigned UGVSystem::ReadRTPriority(
  const std::string & name, const unsigned default_priority, const bool above_control_loop)
{
  // Optional, so a robot description written before these existed keeps the defaults.
  const auto it = info_.hardware_parameters.find(name);
  if (it == info_.hardware_parameters.end()) {
    return default_priority;
  }

  const unsigned min_priority = above_control_loop ? 1 : 0;
  const auto priority = ParseRTPriority(it->second);
  if (!priority || *priority < min_priority) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid " << name << " '" << it->second << "', expected a whole number in ["
                          << min_priority << ", " << kMaxRTPriority << "]. Using the default "
                          << default_priority << ".");
    return default_priority;
  }
  if (above_control_loop && *priority <= kDefaultControlLoopPriority) {
    RCLCPP_WARN_STREAM(
      logger_, name << " " << *priority << " is not above the control loop's default FIFO "
                    << kDefaultControlLoopPriority
                    << ". If the control loop runs there, it can wait on this thread.");
  }
  return *priority;
}

int UGVSystem::ReadRTCPU(const std::string & name, const int default_cpu)
{
  const auto it = info_.hardware_parameters.find(name);
  const std::string text = it == info_.hardware_parameters.end() ? std::to_string(default_cpu)
                                                                 : it->second;
  const int cpu_count = get_nprocs_conf();

  const auto cpu = ParseCPU(text, cpu_count);
  if (!cpu) {
    RCLCPP_WARN_STREAM(
      logger_, "Invalid " << name << " '" << text << "', expected -1 or a CPU in [0, "
                          << cpu_count - 1 << "]. Using the default.");
    return default_cpu;
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
    std::bind(&UGVSystem::AreVelocityCommandsNearZero, this),
    std::bind(&UGVSystem::IsRoboteqEStopSet, this),
    std::bind(&UGVSystem::MarkRelatchAfterRefusedReset, this));

  RCLCPP_INFO(logger_, "Successfully configured E-Stop");
}

bool UGVSystem::IsRoboteqEStopSet()
{
  for (const auto & [name, can_id] : canopen_settings_.driver_can_ids) {
    try {
      if (robot_driver_->GetData(name).GetFaultFlag().IsEmergencyStopSet()) {
        return true;
      }
    } catch (const std::exception &) {
      // Not a driver of this robot.
    }
  }
  return false;
}

void UGVSystem::DestroyComponents()
{
  // Order matters: close the e-stop (no reset after it) -> ROS interface (joins its executor, so
  // no service callback runs into a stopped GPIO monitor) -> GPIO monitor and watchdog -> e-stop
  // -> driver -> GPIO controller, whose destruction drives VMOT_ON and DRIVER_EN to their init
  // values, so it must outlive the CANopen master.
  if (e_stop_) {
    try {
      e_stop_->Close();
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(logger_, "Closing the E-stop failed: " << e.what());
    }
  }
  {
    std::lock_guard<std::mutex> lck(ros_interface_for_callbacks_mtx_);
    ros_interface_for_callbacks_ = nullptr;
  }
  system_ros_interface_.reset();
  if (gpio_controller_) {
    gpio_controller_->Stop();
  }
  e_stop_.reset();

  if (robot_driver_) {
    robot_driver_->Deinitialize();
    robot_driver_.reset();
  }

  gpio_controller_.reset();
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

  // A refused reset of a latched E-stop marks its latch through MarkRelatchAfterRefusedReset.
  e_stop_->ResetEStop();
  // The reset turns the Roboteq emergency stop off, which enables the motor torque again.
  SetTorqueRequest(TorqueRequest::NONE);
}

void UGVSystem::MarkRelatchAfterRefusedReset()
{
  // Marked before the write, like a torque disable: the fault flags of the latch can arrive
  // before the write returns.
  SetTorqueRequest(TorqueRequest::RELATCH_PENDING);
}

void UGVSystem::SetTorqueRequest(const TorqueRequest request)
{
  diagnostics_state_.torque_request_ns.store(EStopDiagnostics::SteadyNowNs());
  diagnostics_state_.torque_request.store(request);
}

void UGVSystem::UpdateTorqueRequest(const bool emergency_stop_set)
{
  // The fault flags arrive in a TPDO up to 50 ms after a torque change was written, and on a
  // Panther the two drivers report it at different times. A request ends when every driver
  // confirms it, or after kTorqueRequestTimeout when they never do. A compare-exchange keeps a
  // request the service writes meanwhile.
  auto request = diagnostics_state_.torque_request.load();
  if (request == TorqueRequest::NONE) {
    return;
  }

  const bool timed_out = EStopDiagnostics::SteadyNowNs() -
                           diagnostics_state_.torque_request_ns.load() >
                         std::chrono::nanoseconds(kTorqueRequestTimeout).count();
  auto next = request;
  switch (request) {
    case TorqueRequest::DISABLE_PENDING:
    case TorqueRequest::RELATCH_PENDING:
      if (emergency_stop_set) {
        next = request == TorqueRequest::DISABLE_PENDING ? TorqueRequest::DISABLED
                                                         : TorqueRequest::RELATCHED;
      } else if (timed_out) {
        next = TorqueRequest::NONE;
      }
      break;
    case TorqueRequest::DISABLED:
    case TorqueRequest::RELATCHED:
      // The torque came back without an enable request, the request is over.
      if (!emergency_stop_set) {
        next = TorqueRequest::NONE;
      }
      break;
    case TorqueRequest::ENABLE_PENDING:
      if (!emergency_stop_set || timed_out) {
        next = TorqueRequest::NONE;
      }
      break;
    default:
      break;
  }

  if (
    next == request || !diagnostics_state_.torque_request.compare_exchange_strong(request, next)) {
    return;
  }

  // A torque toggle clears the latched errors once the enable is confirmed or timed out, so a
  // stale sample from before the enable does not latch them again.
  if (request == TorqueRequest::ENABLE_PENDING) {
    roboteq_error_filter_->SetClearErrorsFlag();
  }
  // The torque came back without an enable. A motor controller fault latched only by the disable
  // ends with it; a real fault next to it stays.
  const bool only_the_disable_latched =
    !roboteq_error_filter_->IsUnexcusedError(ErrorsFilterIds::ROBOTEQ_DRIVER) &&
    !roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS) &&
    !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES) &&
    !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);
  if (
    (request == TorqueRequest::DISABLED || request == TorqueRequest::RELATCHED) &&
    only_the_disable_latched) {
    roboteq_error_filter_->SetClearErrorsFlag();
  }
}

bool UGVSystem::IsFlagErrorOnlyTorqueDisable(const DriverData & data) const
{
  if (
    diagnostics_state_.torque_request.load() == TorqueRequest::NONE ||
    !data.GetFaultFlag().IsEmergencyStopSet()) {
    return false;
  }

  auto flag_errors = data.GetFlagErrorMap();
  flag_errors.erase(FaultFlag::kEmergencyStopKey);
  for (const auto & channel : {"channel_1_motor.", "channel_2_motor."}) {
    flag_errors.erase(std::string(channel) + RuntimeError::kEmergencyStopBitKey);
  }
  return std::none_of(
    flag_errors.begin(), flag_errors.end(), [](const auto & flag) { return flag.second; });
}

void UGVSystem::UpdateMotorsState()
{
  try {
    robot_driver_->UpdateMotorsState();
    UpdateHwStates();
    UpdateMotorsStateDataTimedOut();
  } catch (const std::runtime_error & e) {
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, steady_clock_, 5000,
      "An exception occurred while updating motors states: " << e.what());
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

void UGVSystem::UpdateEStopState()
{
  if (robot_driver_->CommunicationError()) {
    RCLCPP_ERROR_THROTTLE(
      logger_, steady_clock_, 5000, "Triggering E-Stop due to robot driver communication error.");
    e_stop_->TriggerEStop();
  }

  const bool e_stop = e_stop_->ReadEStopState();
  system_ros_interface_->PublishEStopStateIfChanged(e_stop);
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

  status.summary(level, message);
}

void UGVSystem::DiagnoseDriversStatus(
  diagnostic_updater::DiagnosticStatusWrapper & status, const std::string & robot_name)
{
  unsigned char level = diagnostic_updater::DiagnosticStatusWrapper::OK;
  std::string message = robot_name + " system status monitoring.";
  std::uint32_t pdo_timeouts = 0;

  try {
    for (const auto & driver : GetDriversDiagnosticData()) {
      const auto & state = driver.data.GetDriverState();
      status.add(driver.name + " voltage (V)", state.GetVoltage());
      status.add(driver.name + " battery current (A)", state.GetCurrent());
      status.add(driver.name + " temperature (\u00B0C)", state.GetTemperature());
      status.add(driver.name + " heatsink temperature (\u00B0C)", state.GetHeatsinkTemperature());
      status.add(
        "Motor current " + driver.right_motor + " (A)",
        driver.data.GetMotorState(MotorChannels::RIGHT).GetCurrent());
      status.add(
        "Motor current " + driver.left_motor + " (A)",
        driver.data.GetMotorState(MotorChannels::LEFT).GetCurrent());
      if (driver.data.GetMotorStatesTimeouts() > 0) {
        status.addf(
          driver.name + " motor state PDO age at the last timeout (ms)",
          "%.1f oldest, %.1f freshest", driver.data.GetMotorStatesTimeoutOldestAgeMs(),
          driver.data.GetMotorStatesTimeoutFreshestAgeMs());
      }
      pdo_timeouts += driver.data.GetMotorStatesTimeouts();

      if (state.GetHeatsinkTemperature() >= kHeatsinkTemperatureWarn) {
        level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
        message = driver.name + " heatsink temperature is close to the overheat limit.";
      }
    }
  } catch (const std::exception & e) {
    status.summary(
      diagnostic_updater::DiagnosticStatusWrapper::STALE,
      "Hardware data unavailable: " + std::string(e.what()));
    return;
  }

  auto & diag = diagnostics_state_;
  const auto now = std::chrono::steady_clock::now();
  // The counters are unsigned: a counter that went backwards adds nothing instead of wrapping.
  diag.pdo_timeout_history.emplace_back(
    now,
    pdo_timeouts >= diag.pdo_timeouts_in_history ? pdo_timeouts - diag.pdo_timeouts_in_history : 0);
  diag.pdo_timeouts_in_history = pdo_timeouts;
  while (!diag.pdo_timeout_history.empty() &&
         now - diag.pdo_timeout_history.front().first > std::chrono::hours(1)) {
    diag.pdo_timeout_history.pop_front();
  }
  std::uint32_t pdo_timeouts_last_hour = 0;
  for (const auto & [time, timeouts] : diag.pdo_timeout_history) {
    pdo_timeouts_last_hour += timeouts;
  }
  status.add("PDO timeouts in the last hour", pdo_timeouts_last_hour);

  const auto read_period_max_us = diag.read_period_max_us.exchange(0, std::memory_order_relaxed);
  if (read_period_max_us > 0) {
    status.add("Read period max since the last report (ms)", read_period_max_us / 1000.0);
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
    const auto torque_request = diagnostics_state_.torque_request.load();
    const bool torque_disabled_by_request = torque_request != TorqueRequest::NONE;
    const bool relatched = torque_request == TorqueRequest::RELATCH_PENDING ||
                           torque_request == TorqueRequest::RELATCHED;

    // A latched e-stop holds the motors through the Roboteq safety stop, so on a parked robot
    // that flag is the expected state, not a warning.
    const bool e_stop_latched = e_stop_diagnostics_ &&
                                e_stop_diagnostics_->GetSource() != EStopLatchSource::RELEASED;
    bool flag_error_not_torque_disable = false;
    std::uint32_t pdo_timeouts = 0;

    for (const auto & driver : GetDriversDiagnosticData()) {
      const auto & data = driver.data;
      const std::string error_prefix = driver.name + " error: ";
      pdo_timeouts += data.GetMotorStatesTimeouts();

      // With the torque disabled on request the emergency stop fault is that request, also when
      // other faults are set next to it.
      const bool torque_disable_fault = torque_disabled_by_request &&
                                        data.GetFaultFlag().IsEmergencyStopSet();
      if (torque_disable_fault && relatched) {
        raise(
          diagnostic_updater::DiagnosticStatusWrapper::WARN,
          "Motor controllers latched again after a refused E-stop reset.");
        status.add(driver.name + " warning: motor torque", "disabled after a refused E-stop reset");
      } else if (torque_disable_fault) {
        raise(
          diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor torque disabled on request.");
        status.add(driver.name + " warning: motor torque", "disabled on request");
      }
      auto flag_errors = data.GetFlagErrorMap();
      if (torque_disable_fault) {
        flag_errors.erase(FaultFlag::kEmergencyStopKey);
        // The controller sets this unnamed runtime bit on both channels while its emergency stop
        // is on, so it comes with the torque disable.
        for (const auto & channel : {"channel_1_motor.", "channel_2_motor."}) {
          const auto key = std::string(channel) + RuntimeError::kEmergencyStopBitKey;
          const auto it = flag_errors.find(key);
          if (it != flag_errors.end() && it->second) {
            status.add(driver.name + " warning: " + key, true);
            flag_errors.erase(it);
          }
        }
      }
      for (const auto & [flag, is_set] : flag_errors) {
        if (is_set) {
          flag_error_not_torque_disable = true;
          raise(
            diagnostic_updater::DiagnosticStatusWrapper::ERROR,
            driver.name + " fault: " + flag + ".");
        }
      }
      husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, flag_errors, error_prefix);
      if (data.IsCANError()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, driver.name + " CAN error.");
        status.add(error_prefix + "can_error", true);
      }
      if (data.IsHeartbeatTimeout()) {
        raise(
          diagnostic_updater::DiagnosticStatusWrapper::ERROR, driver.name + " heartbeat timeout.");
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
      add_motor_warning(MotorChannels::RIGHT, driver.right_motor);
      add_motor_warning(MotorChannels::LEFT, driver.left_motor);

      if (data.IsMotorStatesDataTimedOut()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data is late.");
        status.add(warning_prefix + "motor_states_data_timed_out", true);
      }
      if (data.IsDriverStateDataTimedOut()) {
        raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data is late.");
        status.add(warning_prefix + "driver_state_data_timed_out", true);
      }
    }

    const auto new_pdo_timeouts = pdo_timeouts >= diagnostics_state_.pdo_timeouts_reported
                                    ? pdo_timeouts - diagnostics_state_.pdo_timeouts_reported
                                    : 0;
    diagnostics_state_.pdo_timeouts_reported = pdo_timeouts;
    if (new_pdo_timeouts > 0) {
      raise(diagnostic_updater::DiagnosticStatusWrapper::WARN, "Motor controller data was late.");
      status.add("PDO timeouts since the last report", new_pdo_timeouts);
    }

    // The filter latches: these stay set after the cause went away, until it is cleared.
    // A fault that latched while the torque was off and cleared since is still a fault.
    const bool only_torque_disable_latched =
      torque_disabled_by_request && !flag_error_not_torque_disable &&
      roboteq_error_filter_->IsError(ErrorsFilterIds::ROBOTEQ_DRIVER) &&
      !roboteq_error_filter_->IsUnexcusedError(ErrorsFilterIds::ROBOTEQ_DRIVER) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::WRITE_PDO_CMDS) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES) &&
      !roboteq_error_filter_->IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE);
    if (roboteq_error_filter_->IsError() && !only_torque_disable_latched) {
      const auto latched_errors = roboteq_error_filter_->GetErrorDescriptionMap();
      for (const auto & [description, is_set] : latched_errors) {
        if (is_set) {
          raise(diagnostic_updater::DiagnosticStatusWrapper::ERROR, description + ".");
        }
      }
      husarion_ugv_utils::diagnostics::AddKeyValueIfTrue(status, latched_errors);
      status.add(
        "Latched errors clear on",
        "a successful E-stop reset, a torque disable, or a torque enable once the motor "
        "controllers confirm it");
    }
  } catch (const std::exception & e) {
    status.summary(
      diagnostic_updater::DiagnosticStatusWrapper::STALE,
      "Hardware data unavailable: " + std::string(e.what()));
    return;
  }

  status.summary(level, message);
}

void UGVSystem::UpdateReadCycleMax()
{
  timespec now_ts;
  clock_gettime(CLOCK_MONOTONIC, &now_ts);
  auto & last_ts = diagnostics_state_.last_read_ts;
  if (diagnostics_state_.restart_read_period.exchange(false)) {
    last_ts = {0, 0};
  }
  if (last_ts.tv_sec != 0 || last_ts.tv_nsec != 0) {
    const auto read_period_us = std::min<std::int64_t>(
      (now_ts.tv_sec - last_ts.tv_sec) * 1000000LL + (now_ts.tv_nsec - last_ts.tv_nsec) / 1000,
      std::numeric_limits<std::uint32_t>::max());
    const auto read_cycle_us = static_cast<std::uint32_t>(read_period_us);
    auto & read_period_max_us = diagnostics_state_.read_period_max_us;
    auto max_us = read_period_max_us.load(std::memory_order_relaxed);
    while (read_cycle_us > max_us && !read_period_max_us.compare_exchange_weak(
                                       max_us, read_cycle_us, std::memory_order_relaxed)) {
    }
  }
  last_ts = now_ts;
}

void UGVSystem::MotorTorqueEnable(const bool enable)
{
  try {
    {
      // Checked under the write mutex, which a reset takes for its release, so the check and the
      // write are not split by one.
      std::lock_guard<std::mutex> lck_g(*robot_driver_write_mtx_);
      if (!e_stop_->ReadEStopState()) {
        throw std::invalid_argument("Can't enable/disable torque when E-Stop is not triggered.");
      }

      // Marked before the write: an SDO write that times out may still have reached the Roboteq,
      // and the diagnostics must not read the torque disable it caused as a fault.
      if (!enable) {
        SetTorqueRequest(TorqueRequest::DISABLE_PENDING);
        robot_driver_->TurnOnEStop();
      } else {
        robot_driver_->TurnOffEStop();
        SetTorqueRequest(TorqueRequest::ENABLE_PENDING);
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
  } catch (const std::invalid_argument & e) {
    throw std::runtime_error(e.what());
  }
}

}  // namespace husarion_ugv_hardware_interfaces
