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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATION_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATION_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rclcpp/time.hpp"

#include "husarion_ugv_lights/led_components/led_segment.hpp"

namespace husarion_ugv_lights
{

/**
 * @brief Structure describing basic animation, including its type, description and list of segments
 * it will be assigned to
 */
struct AnimationDescription
{
  std::string type;
  std::vector<std::string> segments;
  YAML::Node animation;
};

/**
 * @brief Structure describing a complete LED animation, containing its ID, priority, name, timeout
 * and, a list of animations that will be displayed on LED segments
 */
struct LEDAnimationDescription
{
  std::uint8_t id;
  std::uint8_t priority;
  std::string name;
  float timeout;
  std::vector<AnimationDescription> animations;
};

/**
 * @brief Class representing animation that can be displayed on robot segments
 */
class LEDAnimation
{
public:
  /**
   * @brief Initializes LED animation
   *
   * @param led_animation_description YAML description of the LED animation
   * @param segments This parameter is used to create map of segments used by this LED animation
   * @param init_time Time of creation of the LED animation
   */
  LEDAnimation(
    const LEDAnimationDescription & led_animation_description,
    const std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments,
    const rclcpp::Time & init_time)
  : led_animation_description_(led_animation_description),
    init_time_(init_time),
    repeating_(false),
    param_("")
  {
    for (const auto & animation : led_animation_description_.animations) {
      for (const auto & segment : animation.segments) {
        if (segments.find(segment) == segments.end()) {
          throw std::runtime_error("No segment with name: " + segment + ".");
        }
        animation_segments_.push_back(segments.at(segment));
      }
    }
  }

  ~LEDAnimation() {}

  std::string GetName() const { return led_animation_description_.name; }
  std::uint8_t GetPriority() const { return led_animation_description_.priority; }
  std::vector<AnimationDescription> GetAnimations() const
  {
    return led_animation_description_.animations;
  }
  rclcpp::Time GetInitTime() const { return init_time_; }
  float GetTimeout() const { return led_animation_description_.timeout; }

  bool IsRepeating() const { return repeating_; }
  std::string GetParam() const { return param_; }

  void SetRepeating(const bool value) { repeating_ = value; }
  void SetParam(const std::string & param) { param_ = param; }

  static constexpr std::uint8_t kDefaultPriority = 3;
  static constexpr float kDefaultTimeout = 120.0f;
  static constexpr std::array<std::uint8_t, 4> kValidPriorities = {0, 1, 2, 3};

private:
  const LEDAnimationDescription led_animation_description_;
  rclcpp::Time init_time_;

  bool repeating_;
  std::string param_;
  std::vector<std::shared_ptr<LEDSegment>> animation_segments_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATION_HPP_
