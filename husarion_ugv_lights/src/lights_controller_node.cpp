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

#include "husarion_ugv_lights/lights_controller_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "husarion_ugv_msgs/srv/set_led_animation.hpp"

#include "husarion_ugv_lights/led_components/led_animation.hpp"
#include "husarion_ugv_lights/led_components/led_panel.hpp"
#include "husarion_ugv_lights/led_components/led_segment.hpp"
#include "husarion_ugv_lights/led_components/segment_converter.hpp"
#include "husarion_ugv_lights/lights_controller_parameters.hpp"
#include "husarion_ugv_utils/ros_utils.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

LightsControllerNode::LightsControllerNode(const rclcpp::NodeOptions & options)
: Node("lights_controller", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing.");

  using namespace std::placeholders;

  this->param_listener_ =
    std::make_shared<lights_controller::ParamListener>(this->get_node_parameters_interface());
  this->params_ = this->param_listener_->get_params();

  const auto animations_config_path = this->params_.animations_config_path;
  const auto user_led_animations_path = this->params_.user_led_animations_path;
  const float controller_freq = static_cast<float>(this->params_.controller_frequency);

  YAML::Node led_config_desc = YAML::LoadFile(animations_config_path);

  InitializeLEDPanels(led_config_desc["panels"]);
  InitializeLEDSegments(led_config_desc["segments"], controller_freq);
  InitializeLEDSegmentsMap(led_config_desc["segments_map"]);
  LoadDefaultAnimations(led_config_desc["led_animations"]);

  if (user_led_animations_path != "") {
    LoadUserAnimations(user_led_animations_path);
  }

  segment_converter_ = std::make_shared<SegmentConverter>();

  set_led_animation_server_ = this->create_service<SetLEDAnimationSrv>(
    "lights/set_animation", std::bind(&LightsControllerNode::SetLEDAnimationCB, this, _1, _2));

  controller_timer_ = this->create_wall_timer(
    std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / controller_freq)),
    std::bind(&LightsControllerNode::ControllerTimerCB, this));

  RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void LightsControllerNode::InitializeLEDPanels(const YAML::Node & panels_description)
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing LED panels.");

  for (auto & panel : panels_description.as<std::vector<YAML::Node>>()) {
    const auto channel = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(panel, "channel");
    const auto number_of_leds = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(
      panel, "number_of_leds");

    const auto result = led_panels_.emplace(channel, std::make_unique<LEDPanel>(number_of_leds));
    if (!result.second) {
      throw std::runtime_error(
        "Multiple panels with channel nr '" + std::to_string(channel) + "' found.");
    }

    const auto pub_result = panel_publishers_.emplace(
      channel,
      this->create_publisher<ImageMsg>("lights/channel_" + std::to_string(channel) + "_frame", 10));
    if (!pub_result.second) {
      throw std::runtime_error(
        "Multiple panel publishers for channel nr '" + std::to_string(channel) + "' found.");
    }

    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "Initialized panel with channel no. " << channel << ".");
  }
}

void LightsControllerNode::InitializeLEDSegments(
  const YAML::Node & segments_description, const float controller_freq)
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing LED segments.");

  for (auto & segment : segments_description.as<std::vector<YAML::Node>>()) {
    const auto segment_name = husarion_ugv_utils::GetYAMLKeyValue<std::string>(segment, "name");

    try {
      const auto result = segments_.emplace(
        segment_name, std::make_shared<LEDSegment>(segment, controller_freq));
      if (!result.second) {
        throw std::runtime_error("Multiple segments with given name found.");
      }
    } catch (const std::runtime_error & e) {
      throw std::runtime_error(
        "Failed to initialize '" + segment_name + "' segment: " + std::string(e.what()));
    } catch (const std::invalid_argument & e) {
      throw std::runtime_error(
        "Failed to initialize '" + segment_name + "' segment: " + std::string(e.what()));
    }

    RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized '" << segment_name << "' segment.");
  }
}

void LightsControllerNode::InitializeLEDSegmentsMap(const YAML::Node & segments_map_description)
{
  RCLCPP_DEBUG(this->get_logger(), "Initializing LED segments map.");

  for (const auto & key : segments_map_description) {
    const auto name = key.first.as<std::string>();
    const auto value = key.second.as<std::vector<std::string>>();
    segments_map_.emplace(name, value);
  }

  RCLCPP_DEBUG(this->get_logger(), "Initialized LED segments map.");
}

void LightsControllerNode::LoadDefaultAnimations(const YAML::Node & animations_description)
{
  RCLCPP_DEBUG(this->get_logger(), "Loading default animations.");

  for (auto & animation_description : animations_description.as<std::vector<YAML::Node>>()) {
    LoadAnimation(animation_description);
  }

  RCLCPP_INFO(this->get_logger(), "Loaded default animations.");
}

void LightsControllerNode::LoadUserAnimations(const std::string & user_led_animations_path)
{
  RCLCPP_DEBUG(this->get_logger(), "Loading user's animations.");

  try {
    YAML::Node user_led_animations = YAML::LoadFile(user_led_animations_path);
    auto user_animations = husarion_ugv_utils::GetYAMLKeyValue<std::vector<YAML::Node>>(
      user_led_animations, "user_animations");

    for (auto & animation_description : user_animations) {
      try {
        auto id = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(animation_description, "id");
        if (id < 20) {
          throw std::runtime_error("Animation ID must be greater than 19.");
        }

        auto priority = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(
          animation_description, "priority", LEDAnimation::kDefaultPriority);
        if (priority == 0) {
          throw std::runtime_error("User animation can not have priority 0.");
        }

        LoadAnimation(animation_description);
      } catch (const std::runtime_error & e) {
        RCLCPP_WARN_STREAM(
          this->get_logger(), "Skipping user animation that failed to load: " << e.what());
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Failed to load user animations: " << e.what());
  }

  RCLCPP_INFO(this->get_logger(), "Loaded user's animations.");
}

void LightsControllerNode::LoadAnimation(const YAML::Node & animation_description)
{
  LEDAnimationDescription led_animation_desc;

  try {
    led_animation_desc.id = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(
      animation_description, "id");
    led_animation_desc.name = husarion_ugv_utils::GetYAMLKeyValue<std::string>(
      animation_description, "name", "ANIMATION_" + std::to_string(led_animation_desc.id));
    led_animation_desc.priority = husarion_ugv_utils::GetYAMLKeyValue<std::uint8_t>(
      animation_description, "priority", LEDAnimation::kDefaultPriority);
    led_animation_desc.timeout = husarion_ugv_utils::GetYAMLKeyValue<float>(
      animation_description, "timeout", LEDAnimation::kDefaultTimeout);

    if (
      std::find(
        LEDAnimation::kValidPriorities.begin(), LEDAnimation::kValidPriorities.end(),
        led_animation_desc.priority) == LEDAnimation::kValidPriorities.end()) {
      throw std::runtime_error("Invalid LED animation priority.");
    }

    auto animations = husarion_ugv_utils::GetYAMLKeyValue<std::vector<YAML::Node>>(
      animation_description, "animations");
    for (auto & animation : animations) {
      AnimationDescription animation_desc;
      animation_desc.type = husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation, "type");
      animation_desc.animation = husarion_ugv_utils::GetYAMLKeyValue<YAML::Node>(
        animation, "animation");

      auto segments_group = husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation, "segments");
      animation_desc.segments = segments_map_.at(segments_group);

      led_animation_desc.animations.push_back(animation_desc);
    }

    const auto result = animations_descriptions_.emplace(led_animation_desc.id, led_animation_desc);
    if (!result.second) {
      throw std::runtime_error("Animation with given ID already exists.");
    }

  } catch (const std::runtime_error & e) {
    throw std::runtime_error(
      "Failed to load '" + led_animation_desc.name + "' animation: " + std::string(e.what()));
  }
}

void LightsControllerNode::SetLEDAnimationCB(
  const SetLEDAnimationSrv::Request::SharedPtr & request,
  SetLEDAnimationSrv::Response::SharedPtr response)
{
  try {
    AddAnimationToLayer(request->animation.id, request->repeating, request->animation.param);
    response->success = true;
  } catch (const std::exception & e) {
    response->success = false;
    response->message = e.what();
  }
}

void LightsControllerNode::PublishPanelFrame(const std::size_t channel)
{
  auto panel = led_panels_.at(channel);
  const auto number_of_leds = panel->GetNumberOfLeds();

  ImageMsg::UniquePtr image(new ImageMsg);
  image->header.frame_id = husarion_ugv_utils::ros::AddNamespaceToFrameID(
    "lights_channel_" + std::to_string(channel) + "_link", std::string(this->get_namespace()));
  image->header.stamp = this->get_clock()->now();
  image->encoding = "rgba8";
  image->height = 1;
  image->width = number_of_leds;
  image->step = number_of_leds * 4;
  image->data = panel->GetFrame();

  panel_publishers_.at(channel)->publish(std::move(image));
}

void LightsControllerNode::ControllerTimerCB() { UpdateAndPublishAnimation(); }

void LightsControllerNode::UpdateAndPublishAnimation()
{
  std::vector<std::shared_ptr<LEDSegment>> segments_vec;

  for (auto & [segment_name, segment] : segments_) {
    try {
      if (segment->HasAnimation()) {
        segment->UpdateAnimation();
      }
    } catch (const std::runtime_error & e) {
      RCLCPP_WARN_STREAM(
        this->get_logger(),
        "Failed to update animation on " << segment_name << " segment: " << e.what());
    }
  }

  try {
    segment_converter_->Convert(segments_, led_panels_);
  } catch (const std::runtime_error & e) {
    RCLCPP_ERROR(this->get_logger(), e.what());
    return;
  }

  for (auto & panel : led_panels_) {
    PublishPanelFrame(panel.first);
  }
}

void LightsControllerNode::AddAnimationToLayer(
  const std::size_t animation_id, const bool repeating, const std::string & param)
{
  if (animations_descriptions_.find(animation_id) == animations_descriptions_.end()) {
    throw std::runtime_error("No animation with ID: " + std::to_string(animation_id));
  }

  auto animation_description = animations_descriptions_.at(animation_id);
  auto animation = std::make_shared<LEDAnimation>(
    animation_description, segments_, this->get_clock()->now());
  animation->SetRepeating(repeating);
  animation->SetParam(param);

  SetLEDAnimation(animation);
}

void LightsControllerNode::SetLEDAnimation(const std::shared_ptr<LEDAnimation> & led_animation)
{
  const auto animations = led_animation->GetAnimations();
  for (auto & animation : animations) {
    for (auto & segment : animation.segments) {
      if (segments_.find(segment) == segments_.end()) {
        throw std::runtime_error("No segment with name: " + segment);
      }

      try {
        segments_.at(segment)->SetAnimation(
          animation.type, animation.animation, led_animation->IsRepeating(),
          led_animation->GetPriority(), led_animation->GetParam());
      } catch (const std::runtime_error & e) {
        throw std::runtime_error(
          "Failed to set '" + led_animation->GetName() + "' animation: " + std::string(e.what()));
      }
    }
  }
}

}  // namespace husarion_ugv_lights

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(husarion_ugv_lights::LightsControllerNode)
