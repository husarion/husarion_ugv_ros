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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_SEGMENT_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_SEGMENT_HPP_

#include <cstdint>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "pluginlib/class_loader.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"

namespace husarion_ugv_lights
{

/**
 * @brief Class that represents virtual LED segment of the robot
 */
class LEDSegment
{
public:
  /**
   * @brief Parses basic parameters of the LED segment
   *
   * @param segment_description YAML description of the segment. Must contain given keys:
   * - led_range (string) - two numbers with hyphen in between, eg.: '0-45',
   * - channel (int) - id of physical LED channel to which segment is assigned.
   * @param controller_frequency frequency at which animation will be updated.
   *
   * @exception std::runtime_error or std::invalid_argument if missing required description key or
   * key couldn't be parsed
   */
  LEDSegment(const YAML::Node & segment_description, const float controller_frequency);

  ~LEDSegment();

  /**
   * @brief Overwrite current animation
   *
   * @param animation_description YAML description of the animation. Must contain 'type' key -
   * pluginlib animation type
   * @param repeating if true, will set the default animation for the panel
   *
   * @exception std::runtime_error if 'type' key is missing, given pluginlib fails to load or
   * animation fails to initialize
   */
  void SetAnimation(
    const std::string & type, const YAML::Node & animation_description, const bool repeating,
    const std::string & param = "");

  /**
   * @brief Update animation frame
   *
   * @param param optional parameter to pass to animation when updating
   *
   * @exception std::runtime_error if fails to update animation
   */
  void UpdateAnimation();

  /**
   * @brief Check if animation is finished. This does not return state of the default animation
   *
   * @return True if animation is finished, false otherwise
   */
  bool IsAnimationFinished() const { return animation_finished_; }

  /**
   * @brief Get current animation frame
   *
   * @return Current animation frame or default animation frame if it was defined and the main
   * animation is finished
   * @exception std::runtime_error if segment animation is not defined
   */
  std::vector<std::uint8_t> GetAnimationFrame() const;

  /**
   * @brief Get current animation progress
   *
   * @return Current animation progress
   *
   * @exception std::runtime_error if segment animation is not defined
   */
  float GetAnimationProgress() const;

  /**
   * @brief Reset current animation
   *
   * @exception std::runtime_error if segment animation is not defined
   */
  void ResetAnimation();

  /**
   * @brief Get current animation brightness
   *
   * @exception std::runtime_error if segment animation is not defined
   */
  std::uint8_t GetAnimationBrightness() const;

  std::size_t GetFirstLEDPosition() const;

  std::size_t GetChannel() const { return channel_; }

  bool HasAnimation() const { return animation_ || default_animation_; }

protected:
  std::shared_ptr<husarion_ugv_lights::Animation> animation_;
  std::shared_ptr<husarion_ugv_lights::Animation> default_animation_;

private:
  const float controller_frequency_;
  bool invert_led_order_ = false;
  bool animation_finished_ = true;
  std::size_t channel_;
  std::size_t first_led_iterator_;
  std::size_t last_led_iterator_;
  std::size_t num_led_;

  std::shared_ptr<pluginlib::ClassLoader<husarion_ugv_lights::Animation>> animation_loader_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_SEGMENT_HPP_
