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

#ifndef HUSARION_UGV_LIGHTS_HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
#define HUSARION_UGV_LIGHTS_HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "husarion_ugv_msgs/msg/led_output_state.hpp"
#include "husarion_ugv_msgs/srv/set_led_brightness.hpp"

#include "husarion_ugv_lights/apa102.hpp"
#include "husarion_ugv_lights/lights_driver_parameters.hpp"

namespace husarion_ugv_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLEDBrightnessSrv = husarion_ugv_msgs::srv::SetLEDBrightness;
using LEDOutputStateMsg = husarion_ugv_msgs::msg::LEDOutputState;

/**
 * @brief Class for controlling APA102 LEDs based on a ROS Image topic.
 */
class LightsDriverNode : public rclcpp::Node
{
public:
  LightsDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Constructs the node on the given LED channels, which tests use to pass mocks. A null
   * channel opens its SPI device, as the default constructor does.
   */
  LightsDriverNode(
    APA102Interface::SharedPtr channel_1, APA102Interface::SharedPtr channel_2,
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~LightsDriverNode();

  /**
   * @brief Throws std::invalid_argument when the SPI parameters cannot work: a clock below the
   * lower limit, or a transfer size too small for the LED count.
   */
  static void ValidateSPIParams(const lights_driver::Params & params);

protected:
  /**
   * @brief Clears all LEDs on both channels.
   */
  void ClearLEDs();

  /**
   * @brief Writes the last accepted frame of each channel to the strips again.
   */
  void RewriteLastFrames();

  void CacheFrame(const std::vector<std::uint8_t> & frame, const std::string & panel_name);

  /**
   * @brief Callback of the lights/enable service, which switches the physical LED output.
   *
   * @param req Request with the desired state.
   * @param res Response of the service.
   */
  void EnableLEDOutputCB(
    const SetBoolSrv::Request::SharedPtr & req, SetBoolSrv::Response::SharedPtr res);

  /**
   * @brief Publishes the current output state on lights/output_state.
   */
  void PublishOutputState();

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

  /**
   * @brief Asks the hardware for LED control until it is granted, then cancels its timer.
   */
  void InitializationTimerCB();

  /**
   * @brief Registers the hook that clears the LEDs at shutdown.
   */
  void RegisterShutdownHook();

  void DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status);

  enum class FrameDropReason : std::uint8_t {
    NONE = 0,
    TIMEOUT,
    PAST,
    ENCODING,
    HEIGHT,
    WIDTH,
    DATA_SIZE,
    WRITE,
  };

  static std::string FrameDropReasonToString(const FrameDropReason reason);

  // A few late frames right after start or under load are normal, so the task warns only above
  // this many dropped frames per report.
  static constexpr std::uint32_t kFramesDroppedWarnThreshold = 2;

  void SetBrightnessCB(
    const SetLEDBrightnessSrv::Request::SharedPtr & request,
    SetLEDBrightnessSrv::Response::SharedPtr response);

  int channel_1_num_led_;
  int channel_2_num_led_;
  double frame_timeout_;
  bool led_control_granted_;
  bool led_control_pending_;
  bool led_output_enabled_;
  float global_brightness_;

  rclcpp::Time channel_1_ts_;
  rclcpp::Time channel_2_ts_;

  unsigned initialization_attempt_;
  rclcpp::Time led_control_call_time_;

  rclcpp::Client<SetBoolSrv>::SharedPtr enable_led_control_client_;
  rclcpp::TimerBase::SharedPtr initialization_timer_;

  std::vector<std::uint8_t> channel_1_frame_;
  std::vector<std::uint8_t> channel_2_frame_;

  // Counted per dropped frame and reported by the diagnostic task, instead of a diagnostic
  // broadcast per frame.
  std::uint32_t frames_dropped_{0};
  FrameDropReason last_frame_drop_reason_{FrameDropReason::NONE};
  bool write_failed_{false};

private:
  void OnShutdown();

  /**
   * @brief Asks for LED control once the service is available, without blocking the executor.
   * Used instead of the bounded attempts when led_control_retry_forever is set.
   */
  void RetryLEDControl();

  /**
   * @brief Toggles LED control ON or OFF.
   *
   * @param enable True to enable LED control, false to disable.
   */
  void ToggleLEDControl(const bool enable);

  /**
   * @brief Sends the LED control request without checking that the service is available.
   */
  void SendLEDControlRequest(const bool enable);

  static constexpr unsigned kMaxInitializationAttempts = 3;
  static constexpr unsigned kServiceResponseTimeout = 3;
  static constexpr unsigned kWaitForServiceTimeout = 3;
  static constexpr std::uint64_t kDefaultSPICoreClockHz = 500000000;
  static constexpr std::int64_t kMinSPISpeedHz = 100000;
  // Also the upper bound of spi_speed_hz in lights_driver_parameters.yaml.
  static constexpr std::int64_t kMaxSPISpeedHz = 20000000;

  rclcpp::OnShutdownCallbackHandle shutdown_callback_handle_;

  APA102Interface::SharedPtr channel_1_;
  APA102Interface::SharedPtr channel_2_;

  std::shared_ptr<lights_driver::ParamListener> param_listener_;
  lights_driver::Params params_;

  rclcpp::Service<SetLEDBrightnessSrv>::SharedPtr set_brightness_server_;
  rclcpp::Service<SetBoolSrv>::SharedPtr enable_led_output_server_;
  rclcpp::Publisher<LEDOutputStateMsg>::SharedPtr output_state_pub_;

  rclcpp::CallbackGroup::SharedPtr client_callback_group_;

  rclcpp::Subscription<ImageMsg>::SharedPtr channel_1_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr channel_2_sub_;

  diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_HUSARION_UGV_LIGHTS_LIGHTS_DRIVER_NODE_HPP_
