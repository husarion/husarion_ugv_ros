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

#include "husarion_ugv_lights/lights_driver_node.hpp"

#include <cstdint>
#include <filesystem>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "husarion_ugv_msgs/msg/led_output_state.hpp"
#include "husarion_ugv_msgs/srv/set_led_brightness.hpp"

#include "husarion_ugv_lights/apa102.hpp"
#include "husarion_ugv_lights/lights_controller_parameters.hpp"

namespace husarion_ugv_lights
{

using std::placeholders::_1;
using std::placeholders::_2;

LightsDriverNode::LightsDriverNode(const rclcpp::NodeOptions & options)
: LightsDriverNode(nullptr, nullptr, options)
{
}

LightsDriverNode::LightsDriverNode(
  APA102Interface::SharedPtr channel_1, APA102Interface::SharedPtr channel_2,
  const rclcpp::NodeOptions & options)
: Node("lights_driver", options),
  led_control_granted_(false),
  led_control_pending_(false),
  led_output_enabled_(true),
  global_brightness_(1.0f),
  initialization_attempt_(0),
  channel_1_(
    channel_1 ? channel_1
              : std::make_shared<APA102>(std::make_shared<SPIDevice>(), "/dev/spiled-channel1")),
  channel_2_(
    channel_2 ? channel_2
              : std::make_shared<APA102>(std::make_shared<SPIDevice>(), "/dev/spiled-channel2")),
  diagnostic_updater_(this)
{
  RCLCPP_INFO(this->get_logger(), "Constructing node.");

  this->param_listener_ =
    std::make_shared<lights_driver::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  frame_timeout_ = this->params_.frame_timeout;
  channel_1_num_led_ = this->params_.channel_1_num_led;
  channel_2_num_led_ = this->params_.channel_2_num_led;

  led_output_enabled_ = this->params_.led_output_enabled;

  global_brightness_ = static_cast<float>(this->params_.global_brightness);
  channel_1_->SetGlobalBrightness(global_brightness_);
  channel_2_->SetGlobalBrightness(global_brightness_);

  client_callback_group_ =
    this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  enable_led_control_client_ = this->create_client<SetBoolSrv>(
    "hardware/led_control_enable", rclcpp::ServicesQoS(), client_callback_group_);

  set_brightness_server_ = this->create_service<SetLEDBrightnessSrv>(
    "lights/set_brightness", std::bind(&LightsDriverNode::SetBrightnessCB, this, _1, _2));

  enable_led_output_server_ = this->create_service<SetBoolSrv>(
    "lights/enable", std::bind(&LightsDriverNode::EnableLEDOutputCB, this, _1, _2));

  output_state_pub_ = this->create_publisher<LEDOutputStateMsg>(
    "lights/output_state", rclcpp::QoS(1).transient_local());
  PublishOutputState();

  // running at 10 Hz
  initialization_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100), std::bind(&LightsDriverNode::InitializationTimerCB, this));

  channel_1_ts_ = this->get_clock()->now();
  channel_1_sub_ = this->create_subscription<ImageMsg>(
    "lights/channel_1_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
      FrameCB(msg, channel_1_, channel_1_ts_, "channel_1");
      channel_1_ts_ = msg->header.stamp;
    });

  channel_2_ts_ = this->get_clock()->now();
  channel_2_sub_ = this->create_subscription<ImageMsg>(
    "lights/channel_2_frame", 5, [&](const ImageMsg::UniquePtr & msg) {
      FrameCB(msg, channel_2_, channel_2_ts_, "channel_2");
      channel_2_ts_ = msg->header.stamp;
    });

  diagnostic_updater_.setHardwareID("Bumper Lights");
  diagnostic_updater_.add("Lights driver status", this, &LightsDriverNode::DiagnoseLights);

  // Last, because a constructor that throws runs no destructor to remove the hook.
  RegisterShutdownHook();

  RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

LightsDriverNode::~LightsDriverNode()
{
  // The hook captures this, and the node can be unloaded from its container before shutdown.
  this->get_node_base_interface()->get_context()->remove_on_shutdown_callback(
    shutdown_callback_handle_);
}

void LightsDriverNode::RegisterShutdownHook()
{
  shutdown_callback_handle_ =
    this->get_node_base_interface()->get_context()->add_on_shutdown_callback(
      std::bind(&LightsDriverNode::OnShutdown, this));
}

void LightsDriverNode::OnShutdown()
{
  // An exception escaping a shutdown hook aborts the whole container.
  try {
    ClearLEDs();
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Clearing LEDs at shutdown failed: " << e.what());
  }

  // LED control is not released here. on_shutdown hooks run after the context is shut down, when
  // no request can be sent, and a request sent from a pre-shutdown hook was not delivered either,
  // since the context goes down right after it. When the hardware interface stops it resets the
  // LED control pin itself; when only the lights stop, control stays with this computer.
}

void LightsDriverNode::InitializationTimerCB()
{
  if (led_control_granted_) {
    initialization_timer_->cancel();
    return;
  }

  if (led_control_pending_) {
    if (
      this->now() - led_control_call_time_ <=
      rclcpp::Duration(std::chrono::seconds(kServiceResponseTimeout))) {
      return;
    }

    if (this->params_.led_control_retry_forever) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "LED control service response timeout.");
      // The unanswered request would otherwise stay in the client for good, one per resend.
      enable_led_control_client_->prune_pending_requests();
    } else {
      RCLCPP_WARN(this->get_logger(), "LED control service response timeout.");
    }
    led_control_pending_ = false;
  }

  if (this->params_.led_control_retry_forever) {
    RetryLEDControl();
    return;
  }

  if (initialization_attempt_ >= kMaxInitializationAttempts) {
    throw std::runtime_error("Failed to initialize LED driver.");
  }

  ToggleLEDControl(true);
  initialization_attempt_++;
}

void LightsDriverNode::RetryLEDControl()
{
  // Poll instead of calling the blocking wait_for_service: this timer shares the executor with the
  // lights controller and the lights services.
  if (!enable_led_control_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 10000,
      "Waiting for service '%s' to grant LED control.",
      enable_led_control_client_->get_service_name());
    return;
  }

  // A refused request is sent again only after the response timeout, not on every tick.
  if (
    initialization_attempt_ > 0 &&
    this->now() - led_control_call_time_ <=
      rclcpp::Duration(std::chrono::seconds(kServiceResponseTimeout))) {
    return;
  }

  SendLEDControlRequest(true);
  initialization_attempt_++;
}

void LightsDriverNode::ClearLEDs()
{
  channel_1_->SetPanel(std::vector<std::uint8_t>(channel_1_num_led_ * 4, 0));
  channel_2_->SetPanel(std::vector<std::uint8_t>(channel_2_num_led_ * 4, 0));
}

void LightsDriverNode::ToggleLEDControl(const bool enable)
{
  RCLCPP_DEBUG(
    this->get_logger(), "Calling service to toggle LED control to '%s'.",
    enable ? "true" : "false");

  if (!enable_led_control_client_->wait_for_service(std::chrono::seconds(kWaitForServiceTimeout))) {
    RCLCPP_WARN_STREAM(
      this->get_logger(), "Timeout occurred while waiting for service '"
                            << enable_led_control_client_->get_service_name() << "'!");
    return;
  }

  SendLEDControlRequest(enable);
}

void LightsDriverNode::SendLEDControlRequest(const bool enable)
{
  auto request = std::make_shared<SetBoolSrv::Request>();
  request->data = enable;

  enable_led_control_client_->async_send_request(
    request, std::bind(&LightsDriverNode::ToggleLEDControlCB, this, std::placeholders::_1));

  led_control_pending_ = true;
  led_control_call_time_ = this->now();
  RCLCPP_DEBUG(
    this->get_logger(), "Sent request toggling LED control to '%s'.", enable ? "true" : "false");
}

void LightsDriverNode::ToggleLEDControlCB(
  rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future)
{
  RCLCPP_DEBUG(this->get_logger(), "Received response after toggling LED control.");

  const auto result = future.get();

  const auto request = result.first;
  const auto response = result.second;

  if (!response->success) {
    if (this->params_.led_control_retry_forever) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 10000, "Failed to toggle LED control.");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to toggle LED control.");
    }
    led_control_pending_ = false;
    return;
  }

  if (request->data == true) {
    led_control_granted_ = true;
    // Every valid frame updates the cache, before the grant too, so even after a late grant it
    // holds the controller's latest frame. An exception escaping this callback would end the
    // container.
    try {
      ClearLEDs();
      if (led_output_enabled_) {
        RewriteLastFrames();
      }
    } catch (const std::exception & e) {
      RCLCPP_WARN_STREAM(this->get_logger(), "Writing the LED strips failed: " << e.what());
    }
    RCLCPP_INFO(this->get_logger(), "LED control granted.");
  } else {
    led_control_granted_ = false;
    RCLCPP_INFO(this->get_logger(), "LED control revoked.");
  }

  led_control_pending_ = false;
  PublishOutputState();
}

void LightsDriverNode::FrameCB(
  const ImageMsg::UniquePtr & msg, const APA102Interface::SharedPtr & panel,
  const rclcpp::Time & last_time, const std::string & panel_name)
{
  std::string message;
  if (
    (this->get_clock()->now() - rclcpp::Time(msg->header.stamp)) >
    rclcpp::Duration::from_seconds(frame_timeout_)) {
    message = "Timeout exceeded, ignoring frame";
  } else if (rclcpp::Time(msg->header.stamp) < last_time) {
    message = "Dropping message from past";
  } else if (msg->encoding != sensor_msgs::image_encodings::RGBA8) {
    message = "Incorrect image encoding ('" + msg->encoding + "')";
  } else if (msg->height != 1) {
    message = "Incorrect image height " + std::to_string(msg->height);
  } else if (
    msg->width != static_cast<std::uint32_t>(
                    panel_name == "channel_1" ? channel_1_num_led_ : channel_2_num_led_)) {
    message = "Incorrect image width " + std::to_string(msg->width);
  } else if (msg->data.size() != static_cast<std::size_t>(msg->width) * 4) {
    // Checked here, because a cached frame is written later from callbacks that must not throw.
    message = "Incorrect image data size " + std::to_string(msg->data.size());
  }

  // A valid frame is cached before the grant too: the controller may publish a static animation
  // only once, and the grant writes the cached frame after clearing the strips.
  if (!led_control_granted_) {
    if (message.empty()) {
      CacheFrame(msg->data, panel_name);
    }
    PanelThrottleWarnLog(
      panel_name, "Waiting for LED control to be granted. Ignoring frame for " + panel_name + "!");
    return;
  }

  if (!message.empty()) {
    auto warn_msg = message + " on " + panel_name + "!";
    PanelThrottleWarnLog(panel_name, warn_msg);
    diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
    return;
  }

  // Cached even while the output is off, so turning it back on shows the current frame.
  CacheFrame(msg->data, panel_name);

  if (!led_output_enabled_) {
    return;
  }

  panel->SetPanel(msg->data);
}

void LightsDriverNode::SetBrightnessCB(
  const SetLEDBrightnessSrv::Request::SharedPtr & req, SetLEDBrightnessSrv::Response::SharedPtr res)
{
  const float brightness = req->data;

  try {
    channel_1_->SetGlobalBrightness(brightness);
    channel_2_->SetGlobalBrightness(brightness);
  } catch (const std::out_of_range & e) {
    res->success = false;
    res->message = "Failed to set brightness: " + std::string(e.what());
    return;
  }

  global_brightness_ = brightness;
  PublishOutputState();

  auto str_bright = std::to_string(brightness);

  // Round string to two decimal points
  str_bright = str_bright.substr(0, str_bright.find(".") + 3);
  res->success = true;
  res->message = "Changed brightness to " + str_bright;
}

void LightsDriverNode::CacheFrame(
  const std::vector<std::uint8_t> & frame, const std::string & panel_name)
{
  (panel_name == "channel_1" ? channel_1_frame_ : channel_2_frame_) = frame;
}

void LightsDriverNode::RewriteLastFrames()
{
  if (!channel_1_frame_.empty()) {
    channel_1_->SetPanel(channel_1_frame_);
  }
  if (!channel_2_frame_.empty()) {
    channel_2_->SetPanel(channel_2_frame_);
  }
}

void LightsDriverNode::PublishOutputState()
{
  LEDOutputStateMsg msg;
  msg.enabled = led_output_enabled_;
  msg.control_granted = led_control_granted_;
  msg.brightness = global_brightness_;
  output_state_pub_->publish(msg);
}

void LightsDriverNode::EnableLEDOutputCB(
  const SetBoolSrv::Request::SharedPtr & req, SetBoolSrv::Response::SharedPtr res)
{
  const bool enable = req->data;

  if (enable == led_output_enabled_) {
    res->success = true;
    res->message = std::string("LED output already ") + (enable ? "enabled" : "disabled");
    return;
  }

  // An APA102 strip keeps showing the last frame it was sent, so turning the output off writes one
  // black frame, and turning it on writes the cached frames. Without LED control nothing may be
  // written; the grant clears the strips anyway.
  if (led_control_granted_) {
    try {
      if (enable) {
        RewriteLastFrames();
      } else {
        ClearLEDs();
      }
    } catch (const std::exception & e) {
      // A failed enable may have lit one channel already. The output stays off, so blank the
      // strips again rather than leave a frame that FrameCB would never replace.
      if (enable) {
        try {
          ClearLEDs();
        } catch (const std::exception &) {
        }
      }
      res->success = false;
      res->message = "Failed to write the LED strips: " + std::string(e.what());
      return;
    }
  }

  led_output_enabled_ = enable;
  PublishOutputState();

  RCLCPP_INFO(this->get_logger(), "LED output %s.", enable ? "enabled" : "disabled");
  res->success = true;
  res->message = std::string("LED output ") + (enable ? "enabled" : "disabled");
}

void LightsDriverNode::PanelThrottleWarnLog(const std::string panel_name, const std::string message)
{
  if (panel_name == "channel_1") {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
  } else if (panel_name == "channel_2") {
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
  }
}

void LightsDriverNode::DiagnoseLights(diagnostic_updater::DiagnosticStatusWrapper & status)
{
  unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::ERROR};
  std::string message{"Driver is not functional!"};
  std::string led_control_status{"NOT_GRANTED"};

  if (led_control_granted_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::OK;
    message = "Driver is fully functional.";
    led_control_status = "GRANTED";
  } else if (led_control_pending_) {
    error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
    message = "Driver is not yet functional!";
    led_control_status = "PENDING";
  }

  status.add("LED control status", led_control_status);
  status.summary(error_level, message);
}

}  // namespace husarion_ugv_lights

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(husarion_ugv_lights::LightsDriverNode)
