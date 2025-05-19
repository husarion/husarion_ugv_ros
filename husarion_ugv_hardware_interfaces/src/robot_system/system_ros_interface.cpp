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

#include "husarion_ugv_hardware_interfaces/robot_system/system_ros_interface.hpp"

#include <memory>
#include <string>
#include <thread>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "realtime_tools/realtime_publisher.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"

namespace husarion_ugv_hardware_interfaces
{

template class ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>;
template class ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>;

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::RegisterService(
  const rclcpp::Node::SharedPtr node, const std::string & service_name,
  rclcpp::CallbackGroup::SharedPtr group, const rclcpp::QoS & qos)
{
  service_ = node->create_service<SrvT>(
    service_name, std::bind(&ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper, this, _1, _2),
    qos, group);
}

template <typename SrvT, typename CallbackT>
void ROSServiceWrapper<SrvT, CallbackT>::CallbackWrapper(
  SrvRequestConstPtr request, SrvResponsePtr response)
{
  try {
    ProccessCallback(request);
    response->success = true;
  } catch (const std::exception & err) {
    response->success = false;
    response->message = err.what();

    RCLCPP_WARN_STREAM(
      rclcpp::get_logger("UGVSystem"),
      "An exception occurred while handling the request: " << err.what());
  }
}

template <>
void ROSServiceWrapper<std_srvs::srv::SetBool, std::function<void(bool)>>::ProccessCallback(
  SrvRequestConstPtr request)
{
  callback_(request->data);
}

template <>
void ROSServiceWrapper<std_srvs::srv::Trigger, std::function<void()>>::ProccessCallback(
  SrvRequestConstPtr /* request */)
{
  callback_();
}

SystemROSInterface::SystemROSInterface(
  const std::string & node_name, const rclcpp::NodeOptions & node_options)
: node_(rclcpp::Node::make_shared(node_name, node_options)), diagnostic_updater_(node_)
{
  RCLCPP_INFO(rclcpp::get_logger("UGVSystem"), "Constructing node.");

  executor_ = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  executor_->add_node(node_);

  executor_thread_ = std::thread([this]() { executor_->spin(); });

  driver_state_publisher_ = node_->create_publisher<RobotDriverStateMsg>(
    "hardware/robot_driver_state", 5);
  realtime_driver_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<RobotDriverStateMsg>>(
      driver_state_publisher_);

  io_state_publisher_ = node_->create_publisher<IOStateMsg>(
    "hardware/io_state", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_io_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<IOStateMsg>>(io_state_publisher_);

  e_stop_state_publisher_ = node_->create_publisher<BoolMsg>(
    "hardware/e_stop", rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
  realtime_e_stop_state_publisher_ =
    std::make_unique<realtime_tools::RealtimePublisher<BoolMsg>>(e_stop_state_publisher_);

  diagnostic_updater_.setHardwareID("UGV System");

  RCLCPP_INFO(rclcpp::get_logger("UGVSystem"), "Node constructed successfully.");
}

SystemROSInterface::~SystemROSInterface()
{
  if (executor_) {
    executor_->cancel();

    if (executor_thread_.joinable()) {
      executor_thread_.join();
    }

    executor_.reset();
  }

  realtime_driver_state_publisher_.reset();
  driver_state_publisher_.reset();
  realtime_io_state_publisher_.reset();
  io_state_publisher_.reset();
  realtime_e_stop_state_publisher_.reset();
  e_stop_state_publisher_.reset();

  service_wrappers_storage_.clear();

  node_.reset();
}

void SystemROSInterface::UpdateMsgErrorFlags(const DriverNames name, const DriverData & data)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;
  auto & driver_state_named = GetDriverStateByName(driver_state, name);

  driver_state.header.stamp = node_->get_clock()->now();

  driver_state_named.state.fault_flag = data.GetFaultFlag().GetMessage();
  driver_state_named.state.script_flag = data.GetScriptFlag().GetMessage();
  driver_state_named.state.channel_1_motor_runtime_error =
    data.GetRuntimeError(RoboteqDriver::kChannel1).GetMessage();
  driver_state_named.state.channel_2_motor_runtime_error =
    data.GetRuntimeError(RoboteqDriver::kChannel2).GetMessage();
}

void SystemROSInterface::UpdateMsgDriversStates(
  const DriverNames name, const RoboteqDriverState & state)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;
  auto & driver_state_named = GetDriverStateByName(driver_state, name);

  driver_state_named.state.voltage = state.GetVoltage();
  driver_state_named.state.current = state.GetCurrent();
  driver_state_named.state.temperature = state.GetTemperature();
  driver_state_named.state.heatsink_temperature = state.GetHeatsinkTemperature();
}

void SystemROSInterface::UpdateMsgErrors(const CANErrors & can_errors)
{
  auto & driver_state = realtime_driver_state_publisher_->msg_;

  driver_state.error = can_errors.error;
  driver_state.write_pdo_cmds_error = can_errors.write_pdo_cmds_error;
  driver_state.read_pdo_motor_states_error = can_errors.read_pdo_motor_states_error;
  driver_state.read_pdo_driver_state_error = can_errors.read_pdo_driver_state_error;

  for (const auto & [name, driver_errors] : can_errors.driver_errors) {
    auto & driver_state_named = GetDriverStateByName(driver_state, name);

    driver_state_named.state.motor_states_data_timed_out =
      driver_errors.motor_states_data_timed_out;
    driver_state_named.state.driver_state_data_timed_out =
      driver_errors.driver_state_data_timed_out;
    driver_state_named.state.can_error = driver_errors.can_error;
    driver_state_named.state.heartbeat_timeout = driver_errors.heartbeat_timeout;
  }
}

void SystemROSInterface::PublishEStopStateMsg(const bool e_stop)
{
  realtime_e_stop_state_publisher_->msg_.data = e_stop;
  if (realtime_e_stop_state_publisher_->trylock()) {
    realtime_e_stop_state_publisher_->unlockAndPublish();
  }
}

void SystemROSInterface::PublishEStopStateIfChanged(const bool e_stop)
{
  if (realtime_e_stop_state_publisher_->msg_.data != e_stop) {
    PublishEStopStateMsg(e_stop);
  }
}

void SystemROSInterface::PublishRobotDriverState()
{
  if (realtime_driver_state_publisher_->trylock()) {
    realtime_driver_state_publisher_->unlockAndPublish();
  }
}

void SystemROSInterface::InitializeAndPublishIOStateMsg(
  const std::unordered_map<GPIOPin, bool> & io_state)
{
  for (const auto & [pin, pin_value] : io_state) {
    UpdateIOStateMsg(pin, pin_value);
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

void SystemROSInterface::PublishIOState(const GPIOInfo & gpio_info)
{
  const bool pin_value = (gpio_info.value == gpiod::line::value::ACTIVE);

  if (!UpdateIOStateMsg(gpio_info.pin, pin_value)) {
    return;
  }

  if (realtime_io_state_publisher_->trylock()) {
    realtime_io_state_publisher_->unlockAndPublish();
  }
}

bool SystemROSInterface::UpdateIOStateMsg(const GPIOPin pin, const bool pin_value)
{
  auto & io_state_msg = realtime_io_state_publisher_->msg_;

  switch (pin) {
    case GPIOPin::AUX_PW_EN:
      io_state_msg.aux_power = pin_value;
      break;
    case GPIOPin::CHRG_SENSE:
      io_state_msg.charger_connected = pin_value;
      break;
    case GPIOPin::CHRG_DISABLE:
      io_state_msg.charger_enabled = !pin_value;
      break;
    case GPIOPin::VDIG_OFF:
      io_state_msg.digital_power = !pin_value;
      break;
    case GPIOPin::FAN_SW:
      io_state_msg.fan = pin_value;
      break;
    case GPIOPin::VMOT_ON:
    case GPIOPin::MOTOR_ON:
      io_state_msg.motor_on = pin_value;
      break;
    case GPIOPin::SHDN_INIT:
      io_state_msg.power_button = pin_value;
      break;
    case GPIOPin::LED_SBC_SEL:
      io_state_msg.led_control = pin_value;
      break;
    default:
      return false;
  }

  return true;
}

rclcpp::CallbackGroup::SharedPtr SystemROSInterface::GetOrCreateNodeCallbackGroup(
  const unsigned group_id, rclcpp::CallbackGroupType callback_group_type)
{
  if (group_id == 0) {
    if (callback_group_type == rclcpp::CallbackGroupType::Reentrant) {
      throw std::runtime_error(
        "Node callback group with id 0 (default group) cannot be of "
        "rclcpp::CallbackGroupType::Reentrant type.");
    }
    return nullptr;  // default node callback group
  }

  const auto search = callback_groups_.find(group_id);
  if (search != callback_groups_.end()) {
    if (search->second->type() != callback_group_type) {
      throw std::runtime_error("Requested node callback group has incorrect type.");
    }
    return search->second;
  }

  auto callback_group = node_->create_callback_group(callback_group_type);
  callback_groups_[group_id] = callback_group;
  return callback_group;
}

DriverStateNamedMsg & SystemROSInterface::GetDriverStateByName(
  RobotDriverStateMsg & robot_driver_state, const DriverNames name)
{
  const auto name_str = DriverNamesToString(name);
  auto & driver_states = robot_driver_state.driver_states;

  auto it = std::find_if(
    driver_states.begin(), driver_states.end(),
    [&name_str](const DriverStateNamedMsg & msg) { return msg.name == name_str; });

  if (it == driver_states.end()) {
    DriverStateNamedMsg driver_state_named;
    driver_state_named.name = name_str;
    driver_states.push_back(driver_state_named);
    it = driver_states.end() - 1;
  }

  return *it;
}

}  // namespace husarion_ugv_hardware_interfaces
