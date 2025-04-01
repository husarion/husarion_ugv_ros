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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_ROS_INTERFACE_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_ROS_INTERFACE_HPP_

#include <any>
#include <functional>
#include <map>
#include <memory>
#include <thread>
#include <unordered_map>

#include <diagnostic_updater/diagnostic_updater.hpp>
#include <rclcpp/rclcpp.hpp>
#include <realtime_tools/realtime_publisher.hpp>

#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "husarion_ugv_msgs/msg/driver_state_named.hpp"
#include "husarion_ugv_msgs/msg/io_state.hpp"
#include "husarion_ugv_msgs/msg/robot_driver_state.hpp"

#include "husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"

using namespace std::placeholders;

namespace husarion_ugv_hardware_interfaces
{

using BoolMsg = std_msgs::msg::Bool;
using RobotDriverStateMsg = husarion_ugv_msgs::msg::RobotDriverState;
using IOStateMsg = husarion_ugv_msgs::msg::IOState;
using DriverStateNamedMsg = husarion_ugv_msgs::msg::DriverStateNamed;
using SetBoolSrv = std_srvs::srv::SetBool;
using TriggerSrv = std_srvs::srv::Trigger;

struct DriverCANErrors
{
  bool motor_states_data_timed_out;
  bool driver_state_data_timed_out;
  bool can_error;
  bool heartbeat_timeout;
};

struct CANErrors
{
  bool error;

  bool write_pdo_cmds_error;
  bool read_pdo_motor_states_error;
  bool read_pdo_driver_state_error;

  std::unordered_map<DriverNames, DriverCANErrors> driver_errors;
};

/**
 * @brief A wrapper class for ROS services that simplifies the creation and management of a service
 * server.
 *
 * @tparam SrvT The type of the service message, derived from a .srv file by the ROS build system.
 * @tparam CallbackT The type of the callback function to be invoked on service request. Currently
 * supported callback function signatures include void() and void(bool).
 */
template <typename SrvT, typename CallbackT>
class ROSServiceWrapper
{
public:
  using SrvSharedPtr = typename rclcpp::Service<SrvT>::SharedPtr;
  using SrvRequestConstPtr = typename SrvT::Request::ConstSharedPtr;
  using SrvResponsePtr = typename SrvT::Response::SharedPtr;

  /**
   * @brief Constructs a ROSServiceWrapper object with the specified callback function.
   *
   * @param callback The callback function to be called whenever the service receives a request.
   * Currently supported callback function signatures include void() and void(bool).
   */
  ROSServiceWrapper(const CallbackT & callback) : callback_(callback) {}

  /**
   * @brief Register and advertises the service with the specified name under the given ROS node.
   *
   * @param node The shared pointer to the ROS node under which the service will be advertised.
   * @param service_name The name of the service. This is the name under which the service will be
   * advertised over ROS.
   * @param group The shared pointer to the node's callback group. Defaults to nullptr, which
   * indicates that the node's default callback group will be used.
   * @param qos The QoS settings for the service. Defaults to ServicesQoS.
   */
  void RegisterService(
    const rclcpp::Node::SharedPtr node, const std::string & service_name,
    rclcpp::CallbackGroup::SharedPtr group = nullptr,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS());

private:
  /**
   * @brief Internal wrapper function for the user-defined callback function.
   *
   * @param request Shared pointer to the service request message.
   * @param response Shared pointer to the service response message.
   */
  void CallbackWrapper(SrvRequestConstPtr request, SrvResponsePtr response);

  /**
   * @brief Specializations of the `ProccessCallback` method for handling specific service request
   * types.
   *
   * These specializations of the `ProccessCallback` method in the `ROSServiceWrapper` template
   * class are designed to handle service requests for `std_srvs::srv::SetBool` and
   * `std_srvs::srv::Trigger` services specifically. They extract the necessary information from the
   * request (if any) and invoke the user-defined callback function accordingly.
   *
   * 1. For `std_srvs::srv::SetBool` service requests, it extracts the boolean data from the request
   * and passes it to the callback function.
   * 2. For `std_srvs::srv::Trigger` service requests, it simply calls the callback function without
   * any parameters since `Trigger` requests do not carry additional data.
   */
  void ProccessCallback(SrvRequestConstPtr request);

  SrvSharedPtr service_;
  CallbackT callback_;
};

/**
 * @brief Class that takes care of additional ROS interface of panther system, such as publishing
 * driver state and providing service for clearing errors
 */
class SystemROSInterface
{
public:
  /**
   * @brief Creates node and executor (in a separate thread), publishers, subscribers and services
   *
   * @param node_name
   * @param node_options
   */
  SystemROSInterface(
    const std::string & node_name,
    const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions());
  ~SystemROSInterface();

  /**
   * @brief Registers a new service server associated with a specific service type and callback on
   * the node.
   *
   * @tparam SrvT The type of ROS service for which the server is being created. This should
   * correspond to a predefined service message type.
   * @tparam CallbackT The type of the callback function that will be executed when the service
   * receives a request. The function's signature should be compatible with the expected request and
   * response types for the given service `SrvT`.
   * @param service_name A string specifying the unique name under which the service will be
   * advertised on the ROS network.
   * @param callback The callback function to be executed when the service receives a request. This
   * function is responsible for processing the incoming request and producing an appropriate
   * response. Currently supported callback function signatures include void() and void(bool).
   * @param group_id An unsigned integer representing the unique identifier of the callback group
   * that the service should be associated with. A value of '0' indicates that the service should
   * use the default node callback group.
   * @param callback_group_type The type of the callback group to be used, expressed as an
   * enumerated value of `rclcpp::CallbackGroupType`. If a new group must be created, this specifies
   * whether it should be `MutuallyExclusive` or `Reentrant`. The default value is
   * `MutuallyExclusive`.
   * @param qos The QoS settings for the service. Defaults to ServicesQoS.
   */
  template <class SrvT, class CallbackT>
  inline void AddService(
    const std::string & service_name, const CallbackT & callback, const unsigned group_id = 0,
    rclcpp::CallbackGroupType callback_group_type = rclcpp::CallbackGroupType::MutuallyExclusive,
    const rclcpp::QoS & qos = rclcpp::ServicesQoS())
  {
    rclcpp::CallbackGroup::SharedPtr callback_group = GetOrCreateNodeCallbackGroup(
      group_id, callback_group_type);

    auto wrapper = std::make_shared<ROSServiceWrapper<SrvT, CallbackT>>(callback);
    wrapper->RegisterService(node_, service_name, callback_group, qos);
    service_wrappers_storage_.push_back(wrapper);
  }

  /**
   * @brief Adds a new diagnostic task.
   *
   * @param name The name of the diagnostic task.
   * @param task_owner A pointer to the object that posses the diagnostic task member function.
   * @param task_fcn A pointer to the diagnostic task member function.
   */
  template <class T>
  inline void AddDiagnosticTask(
    const std::string & name, T * task_owner,
    void (T::*task_fcn)(diagnostic_updater::DiagnosticStatusWrapper &))
  {
    diagnostic_updater_.add(name, task_owner, task_fcn);
  }

  /**
   * @brief Broadcasts a message with the specified level on defined diagnostic tasks.
   *
   * @param level The level of the diagnostic message.
   * @param message The message to be broadcasted.
   */
  inline void BroadcastOnDiagnosticTasks(unsigned char level, const std::string & message)
  {
    diagnostic_updater_.broadcast(level, message);
  }

  /**
   * @brief Updates fault flags, script flags, and runtime errors in the robot driver state msg
   *
   * @param name The name of the driver to update the flags for
   * @param data The data to update the flags with
   */
  void UpdateMsgErrorFlags(const DriverNames name, const DriverData & data);

  /**
   * @brief Updates parameters of the driver (voltage, current and temperature) in robot driver
   * state msg
   *
   * @param name The name of the driver to update the parameters for
   * @param state The data to update the parameters with
   */
  void UpdateMsgDriversStates(const DriverNames name, const RoboteqDriverState & state);

  /**
   * @brief Updates the current state of communication errors and general error state
   */
  void UpdateMsgErrors(const CANErrors & can_errors);

  void PublishEStopStateMsg(const bool e_stop);
  void PublishEStopStateIfChanged(const bool e_stop);
  void PublishRobotDriverState();
  void InitializeAndPublishIOStateMsg(const std::unordered_map<GPIOPin, bool> & io_state);
  void PublishIOState(const GPIOInfo & gpio_info);

protected:
  /**
   * @brief Updates the IOState message and indicates whether its state has changed.
   *
   * @param pin The GPIO pin whose state is to be updated.
   * @param pin_value The new value to be set for the pin. True indicates a high state, and false
   * indicates a low state.
   * @return True if the state update caused a change in the IO state message; returns
   * false otherwise.
   */
  bool UpdateIOStateMsg(const GPIOPin pin, const bool pin_value);

  /**
   * @brief Retrieves an existing callback group from the internal map or creates
   * a new one if it does not exist.
   *
   * When the `group_id` is set to 0 and the `callback_group_type` is set to `MutuallyExclusive`,
   * the method returns a `nullptr`, indicating the usage of the default node callback group.
   *
   * @param group_id The numeric identifier of the callback group. If set to 0, the default
   * node callback group is used.
   * @param callback_group_type The type of the callback group, defined by the
   * `rclcpp::CallbackGroupType` enumeration.
   * @return Shared pointer to the requested callback group, or `nullptr` if the default node
   * callback group is to be used.
   */
  rclcpp::CallbackGroup::SharedPtr GetOrCreateNodeCallbackGroup(
    const unsigned group_id, rclcpp::CallbackGroupType callback_group_type);

  /**
   * @brief Retrieves the driver state message associated with the specified name. If the driver
   * state is not found, it is created.
   *
   * @param robot_driver_state The robot driver state message to retrieve driver state from.
   * @param name The name of the driver state to be retrieved.
   *
   * @return Reference to the driver state message associated with the specified name.
   */
  DriverStateNamedMsg & GetDriverStateByName(
    RobotDriverStateMsg & robot_driver_state, const DriverNames name);

  rclcpp::Node::SharedPtr node_;
  std::unordered_map<unsigned, rclcpp::CallbackGroup::SharedPtr> callback_groups_;
  rclcpp::executors::MultiThreadedExecutor::UniquePtr executor_;
  std::thread executor_thread_;

  rclcpp::Publisher<RobotDriverStateMsg>::SharedPtr driver_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<RobotDriverStateMsg>>
    realtime_driver_state_publisher_;

  rclcpp::Publisher<IOStateMsg>::SharedPtr io_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<IOStateMsg>> realtime_io_state_publisher_;

  rclcpp::Publisher<BoolMsg>::SharedPtr e_stop_state_publisher_;
  std::unique_ptr<realtime_tools::RealtimePublisher<BoolMsg>> realtime_e_stop_state_publisher_;

  diagnostic_updater::Updater diagnostic_updater_;

  std::vector<std::any> service_wrappers_storage_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_SYSTEM_ROS_INTERFACE_HPP_
