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

#include <array>
#include <chrono>
#include <functional>
#include <limits>
#include <memory>
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

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  robot_driver_->Deinitialize();
  robot_driver_.reset();

  gpio_controller_.reset();
  e_stop_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_activate(const rclcpp_lifecycle::State &)
{
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
    "hardware/e_stop_trigger", std::bind(&EStopInterface::TriggerEStop, e_stop_), 1,
    rclcpp::CallbackGroupType::MutuallyExclusive);

  auto e_stop_reset_qos = rclcpp::ServicesQoS();
  e_stop_reset_qos.keep_last(1);
  system_ros_interface_->AddService<TriggerSrv, std::function<void()>>(
    "hardware/e_stop_reset", std::bind(&EStopInterface::ResetEStop, e_stop_), 2,
    rclcpp::CallbackGroupType::MutuallyExclusive, e_stop_reset_qos);

  system_ros_interface_->AddDiagnosticTask(
    std::string("system errors"), this, &UGVSystem::DiagnoseErrors);

  system_ros_interface_->AddDiagnosticTask(
    std::string("system status"), this, &UGVSystem::DiagnoseStatus);

  gpio_controller_->RegisterGPIOEventCallback(
    [this](const auto & state) { system_ros_interface_->PublishIOState(state); });

  const auto io_state = gpio_controller_->QueryControlInterfaceIOStates();
  system_ros_interface_->InitializeAndPublishIOStateMsg(io_state);

  system_ros_interface_->PublishEStopStateMsg(e_stop_->ReadEStopState());

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Deactivation failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  system_ros_interface_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_shutdown(const rclcpp_lifecycle::State &)
{
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Shutdown failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  robot_driver_->Deinitialize();
  robot_driver_.reset();

  gpio_controller_.reset();

  system_ros_interface_.reset();
  e_stop_.reset();

  return CallbackReturn::SUCCESS;
}

CallbackReturn UGVSystem::on_error(const rclcpp_lifecycle::State &)
{
  try {
    e_stop_->TriggerEStop();
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR_STREAM(logger_, "Handling error failed: " << e.what());
    return CallbackReturn::ERROR;
  }

  if (system_ros_interface_) {
    system_ros_interface_->BroadcastOnDiagnosticTasks(
      diagnostic_msgs::msg::DiagnosticStatus::ERROR,
      "An error has occurred during a node state transition.");

    system_ros_interface_.reset();
  }

  robot_driver_->Deinitialize();
  robot_driver_.reset();

  gpio_controller_.reset();
  e_stop_.reset();

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
  UpdateMotorsState();

  if (time >= next_driver_state_update_time_) {
    UpdateDriverState();
    UpdateDriverStateMsg();
    system_ros_interface_->PublishRobotDriverState();
    next_driver_state_update_time_ = time + driver_states_update_period_;
  }

  UpdateEStopState();

  return return_type::OK;
}

return_type UGVSystem::write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */)
{
  const bool e_stop = e_stop_->ReadEStopState();

  if (!e_stop) {
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
  gpio_controller_ = GPIOControllerFactory::CreateGPIOController();
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

void UGVSystem::MotorTorqueEnable(const bool enable)
{
  const bool e_stop = e_stop_->ReadEStopState();
  if (!e_stop) {
    throw std::runtime_error("Can't enable/disable torque when E-Stop is not triggered.");
  }

  try {
    {
      std::lock_guard<std::mutex> lck_g(*robot_driver_write_mtx_);

      if (!enable) {
        robot_driver_->TurnOnEStop();
      } else {
        robot_driver_->TurnOffEStop();
      }
    }

    roboteq_error_filter_->SetClearErrorsFlag();
    roboteq_error_filter_->UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
  } catch (const std::runtime_error & e) {
    auto warn = "An exception occurred while changing the torque state of the motors: " +
                std::string(e.what());
    RCLCPP_WARN_STREAM(logger_, warn);
    throw std::runtime_error(warn);
  }
}

}  // namespace husarion_ugv_hardware_interfaces
