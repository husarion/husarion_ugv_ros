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

#include "husarion_ugv_lights/led_components/segment_queue_layer.hpp"

#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

#include "rclcpp/logging.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"
#include "husarion_ugv_lights/led_components/segment_layer_interface.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

SegmentQueueLayer::SegmentQueueLayer(
  const std::size_t num_led, const bool invert_led_order, const float controller_frequency)
: SegmentLayerInterface(num_led, invert_led_order, controller_frequency)
{
}

void SegmentQueueLayer::SetAnimation(
  const std::string & type, const YAML::Node & animation_description, const bool /* repeating */,
  const std::string & param)
{
  std::shared_ptr<husarion_ugv_lights::Animation> animation;
  try {
    animation = animation_loader_->createSharedInstance(type);
  } catch (pluginlib::PluginlibException & e) {
    throw std::runtime_error("The plugin failed to load. Error: " + std::string(e.what()));
  }

  try {
    animation->Initialize(animation_description, num_led_, controller_frequency_);
    animation->SetParam(param);
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
  } catch (const std::out_of_range & e) {
    throw std::runtime_error("Failed to initialize animation: " + std::string(e.what()));
  }

  if (!animation_finished_) {
    if (animations_queue_.size() < max_queue_size_) {
      animations_queue_.emplace_back(animation);
    } else {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("lights_controller"), "Failed to Set LED animation: Queue is full");
    }

  } else {
    animation_ = std::move(animation);
  }

  animation_finished_ = false;
}

void SegmentQueueLayer::UpdateAnimation()
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  if (animation_->IsFinished()) {
    animation_finished_ = true;
  }

  if (animation_finished_) {
    if (animations_queue_.empty()) {
      animation_.reset();
      return;
    }
    try {
      const auto animation = animations_queue_.front();
      animations_queue_.pop_front();
      animation_ = std::move(animation);
      animation_finished_ = false;
    } catch (const std::runtime_error & e) {
      RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("lights_controller"), "Failed to Set LED animation: " << e.what());
    }
  }

  try {
    animation_->Update();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to update animation: " + std::string(e.what()));
  }
}

}  // namespace husarion_ugv_lights
