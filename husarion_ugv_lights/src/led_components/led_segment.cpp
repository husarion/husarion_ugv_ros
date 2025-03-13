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

#include "husarion_ugv_lights/led_components/led_segment.hpp"

#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

#include "rclcpp/logging.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"
#include "husarion_ugv_lights/led_components/segment_layer.hpp"
#include "husarion_ugv_lights/led_components/segment_layer_interface.hpp"
#include "husarion_ugv_lights/led_components/segment_queue_layer.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

LEDSegment::LEDSegment(const YAML::Node & segment_description, const float controller_frequency)
: controller_frequency_(controller_frequency)
{
  channel_ = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(segment_description, "channel");
  const auto led_range = husarion_ugv_utils::GetYAMLKeyValue<std::string>(
    segment_description, "led_range");

  const std::size_t split_char = led_range.find('-');

  if (split_char == std::string::npos) {
    throw std::invalid_argument("No '-' character found in the led_range expression.");
  }

  try {
    first_led_iterator_ = std::stoi(led_range.substr(0, split_char));
    last_led_iterator_ = std::stoi(led_range.substr(split_char + 1));

    if (first_led_iterator_ > last_led_iterator_) {
      invert_led_order_ = true;
    }
  } catch (const std::invalid_argument & e) {
    throw std::invalid_argument("Error converting string to integer.");
  }

  num_led_ = std::abs(int(last_led_iterator_ - first_led_iterator_)) + 1;

  layers_[ERROR] = std::make_unique<SegmentLayer>(
    num_led_, invert_led_order_, controller_frequency);
  layers_[ALERT] = std::make_unique<SegmentQueueLayer>(
    num_led_, invert_led_order_, controller_frequency);
  layers_[INFO] = std::make_unique<SegmentLayer>(num_led_, invert_led_order_, controller_frequency);
  layers_[STATE] = std::make_unique<SegmentLayer>(
    num_led_, invert_led_order_, controller_frequency);
}

void LEDSegment::SetAnimation(
  const std::string & type, const YAML::Node & animation_description, const bool repeating,
  const std::uint8_t priority, const std::string & param)
{
  std::shared_ptr<husarion_ugv_lights::Animation> animation;

  try {
    if (priority < ERROR || priority > STATE) {
      throw std::invalid_argument("Invalid priority value");
    }
    auto animationPriority = static_cast<AnimationPriority>(priority);
    layers_.at(animationPriority)->SetAnimation(type, animation_description, repeating, param);
  } catch (std::out_of_range & e) {
    throw std::runtime_error("Failed to set animation: " + std::string(e.what()));
  }
}

void LEDSegment::UpdateAnimation()
{
  for (auto & [priority, layer] : layers_) {
    if (layer->HasAnimation()) {
      try {
        layer->UpdateAnimation();
      } catch (const std::runtime_error & e) {
        throw std::runtime_error(
          "Failed to update animation at layer of priority " + std::to_string(priority) + ": " +
          std::string(e.what()));
      }
    }
  }
}

bool LEDSegment::IsAnimationFinished(AnimationPriority layer) const
{
  return layers_.at(layer)->IsAnimationFinished();
}

std::vector<std::uint8_t> LEDSegment::GetAnimationFrame() const { return MergeLayersFrames(); }

std::vector<std::uint8_t> LEDSegment::MergeLayersFrames() const
{
  std::vector<std::uint8_t> output_frame(4 * num_led_, 0);
  for (auto it = layers_.rbegin(); it != layers_.rend();
       ++it) {  // reverse the order of layers for merging
    auto & [priority, layer] = *it;
    if (layer->HasAnimation()) {
      auto overlay_frame = layer->GetAnimationFrame();
      MergeFrames(output_frame, overlay_frame);
    }
  }
  return output_frame;
}

void LEDSegment::MergeFrames(
  std::vector<std::uint8_t> & base_frame, const std::vector<std::uint8_t> & overlay_frame) const
{
  for (std::size_t i = 0; i < num_led_; i++) {
    const auto alpha = overlay_frame[i * 4 + 3];
    for (std::size_t j = 0; j < 3; j++) {
      base_frame[i * 4 + j] =
        (overlay_frame[i * 4 + j] * alpha + base_frame[i * 4 + j] * (255 - alpha)) /
        255;  // value * alpha + background_value * (1 - alpha)
    }
    base_frame[i * 4 + 3] = alpha + base_frame[i * 4 + 3] * (255 - alpha) /
                                      255;  // alpha + (1 - alpha) * background_alpha
  }
}

float LEDSegment::GetAnimationProgress(AnimationPriority layer) const
{
  return layers_.at(layer)->GetAnimationProgress();
}

void LEDSegment::ResetAnimation(AnimationPriority layer)
{
  return layers_.at(layer)->ResetAnimation();
}

std::size_t LEDSegment::GetFirstLEDPosition() const
{
  return (invert_led_order_ ? last_led_iterator_ : first_led_iterator_) * Animation::kRGBAColorLen;
}

bool LEDSegment::LayerHasAnimation(AnimationPriority layer) const
{
  return layers_.at(layer)->HasAnimation();
}

bool LEDSegment::HasAnimation() const
{
  return std::any_of(layers_.begin(), layers_.end(), [](const auto & pair) {
    return pair.second->HasAnimation();
  });  // Can't capture structured bindings inside lambda in c++17 😔😔😔😔
}

}  // namespace husarion_ugv_lights
