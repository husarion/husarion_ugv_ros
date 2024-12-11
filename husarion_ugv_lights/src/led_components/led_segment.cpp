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
#include <stdexcept>

#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/animation/animation.hpp"
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

  animation_loader_ = std::make_shared<pluginlib::ClassLoader<husarion_ugv_lights::Animation>>(
    "husarion_ugv_lights", "husarion_ugv_lights::Animation");
}

LEDSegment::~LEDSegment()
{
  // make sure that animations are destroyed before pluginlib loader
  animation_.reset();
  default_animation_.reset();
  animation_loader_.reset();
}

void LEDSegment::SetAnimation(
  const std::string & type, const YAML::Node & animation_description, const bool repeating,
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

  animation_ = std::move(animation);
  animation_finished_ = false;

  if (repeating) {
    default_animation_ = animation_;
    animation_finished_ = true;
  }
  if (default_animation_) {
    default_animation_->Reset();
  }
}

void LEDSegment::UpdateAnimation()
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  if (animation_->IsFinished()) {
    animation_finished_ = true;
  }

  std::shared_ptr<husarion_ugv_lights::Animation> animation_to_update =
    animation_finished_ && default_animation_ ? default_animation_ : animation_;

  if (animation_finished_ && default_animation_ && default_animation_->IsFinished()) {
    default_animation_->Reset();
  }

  try {
    animation_to_update->Update();
  } catch (const std::runtime_error & e) {
    throw std::runtime_error("Failed to update animation: " + std::string(e.what()));
  }
}

std::vector<std::uint8_t> LEDSegment::GetAnimationFrame() const
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  if (default_animation_ && animation_finished_) {
    return default_animation_->GetFrame(invert_led_order_);
  }

  return animation_->GetFrame(invert_led_order_);
}

float LEDSegment::GetAnimationProgress() const
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  return animation_->GetProgress();
}

void LEDSegment::ResetAnimation()
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  animation_->Reset();
  animation_finished_ = false;
}

std::uint8_t LEDSegment::GetAnimationBrightness() const
{
  if (!animation_) {
    throw std::runtime_error("Segment animation not defined.");
  }

  return animation_->GetBrightness();
}

std::size_t LEDSegment::GetFirstLEDPosition() const
{
  return (invert_led_order_ ? last_led_iterator_ : first_led_iterator_) * Animation::kRGBAColorLen;
}

}  // namespace husarion_ugv_lights
