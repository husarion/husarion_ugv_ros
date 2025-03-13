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

#ifndef HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
#define HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "husarion_ugv_msgs/srv/set_led_brightness.hpp"

#include "husarion_ugv_lights/apa102.hpp"
#include "husarion_ugv_lights/lights_driver_parameters.hpp"

namespace husarion_ugv_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLEDBrightnessSrv = husarion_ugv_msgs::srv::SetLEDBrightness;

/**
 * @brief Class for controlling APA102 LEDs based on a ROS Image topic.
 */
class LightsDriverNode : public rclcpp::Node
{
public:
  LightsDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  LightsDriverNode(
    APA102Interface::SharedPtr channel_1, APA102Interface::SharedPtr channel_2,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

protected:
  /**
   * @brief Clears all LEDs on both channels.
   */
  void ClearLEDs();

  /**
   * @brief Callback to execute when service invoked to toggle LED control returns response.
   *
   * @param future Future object with request and response of the service call.
   */
  void ToggleLEDControlCB(rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future);

  /**
   * @brief Callback to execute when a message with new frame is received.
   *
   * @param msg ROS Image message received.
   * @param panel APA102 panel for which LEDs should be set.
   * @param last_time ROS time of the last message received.
   * @param panel_name name of the panel for which the message was received, used for improved
   * logging. Valid names are: 'channel_1', 'channel_2'.
   */
  void FrameCB(
    const ImageMsg::UniquePtr & msg, const APA102Interface::SharedPtr & panel,
    const rclcpp::Time & last_time, const std::string & panel_name);

  /**
   * @brief Logs a warning message to the panel throttle log. Since this is throttle warning, we
   * need to add panel name condition to log from both panels.
   *
   * @param panel_name name of the panel for which the message was received, used for improved
   * logging. Valid names are: 'channel_1', 'channel_2'.
   * @param message warning message to log.
   */
  void PanelThrottleWarnLog(const std::string panel_name, const std::string message);

  int channel_1_num_led_;
  int channel_2_num_led_;
  double frame_timeout_;
  bool led_control_granted_;
  bool led_control_pending_;

  rclcpp::Time channel_1_ts_;
  rclcpp::Time channel_2_ts_;

private:
  void OnShutdown();

  void InitializationTimerCB();

  /**
   * @brief Toggles LED control ON or OFF.
   *
   * @param enable True to enable LED control, false to disable.
   */
  void ToggleLEDControl(const bool enable);

  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & request,
    SetLEDBrightnessSrv::Response::SharedPtr response);

  void DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status);

  static constexpr unsigned kMaxInitializationAttempts = 3;
  static constexpr unsigned kServiceResponseTimeout = 3;
  static constexpr unsigned kWaitForServiceTimeout = 3;

  unsigned initialization_attempt_;
  rclcpp::Time led_control_call_time_;

  APA102Interface::SharedPtr channel_1_;
  APA102Interface::SharedPtr channel_2_;

  std::shared_ptr<lights_driver::ParamListener> param_listener_;
  lights_driver::Params params_;

  rclcpp::TimerBase::SharedPtr initialization_timer_;

  rclcpp::Client<SetBoolSrv>::SharedPtr enable_led_control_client_;
  rclcpp::Service<SetLEDBrightnessSrv>::SharedPtr set_brightness_server_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  rclcpp::Subscription<ImageMsg>::SharedPtr channel_1_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr channel_2_sub_;

  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
