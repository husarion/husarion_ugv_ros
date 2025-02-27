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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "pluginlib/class_loader.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"

namespace husarion_ugv_lights
{

/**
 * @brief Interface for LED segment layers
 */
class SegmentLayerInterface
{
public:
  /**
   * @brief Parses basic parameters of the LED segment
   *
   * @param num_led number of LEDs
   * @param invert_led_order if true will invert the order of LEDs
   * @param controller_frequency frequency at which animation will be updated.
   *
   * @exception std::runtime_error if pluginlib fails to load animation plugin
   */
  SegmentLayerInterface(
    const std::size_t num_led, const bool invert_led_order, const float controller_frequency)
  : controller_frequency_(controller_frequency),
    invert_led_order_(invert_led_order),
    num_led_(num_led)
  {
    animation_loader_ = std::make_shared<pluginlib::ClassLoader<husarion_ugv_lights::Animation>>(
      "husarion_ugv_lights", "husarion_ugv_lights::Animation");
  }

  virtual ~SegmentLayerInterface()
  {
    // make sure that animations are destroyed before pluginlib loader
    animation_.reset();
    animation_loader_.reset();
  }

  /**
   * @brief Overwrite current animation
   *
   * @param type pluginlib animation type
   * @param animation_description YAML description of the animation. Must contain 'type' key -
   * pluginlib animation type
   * @param repeating if true, will repeat animation for the panel
   * @param param optional parameter to pass to animation when initializing
   *
   * @exception std::runtime_error if failed to set animation
   */
  virtual void SetAnimation(
    const std::string & type, const YAML::Node & animation_description, const bool repeating,
    const std::string & param = "") = 0;

  /**
   * @brief Update animation frame
   *
   * @exception std::runtime_error if failed to update animation
   */
  virtual void UpdateAnimation() = 0;

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
  std::vector<std::uint8_t> GetAnimationFrame() const
  {
    if (animation_finished_ || !animation_) {
      return std::vector<std::uint8_t>(4 * num_led_, 0);
    }

    return animation_->GetFrame(invert_led_order_);
  }

  /**
   * @brief Get current animation progress
   *
   * @return Current animation progress
   *
   * @exception std::runtime_error if segment animation is not defined
   */
  float GetAnimationProgress() const
  {
    if (!animation_) {
      throw std::runtime_error("Segment animation not defined.");
    }

    return animation_->GetProgress();
  }

  /**
   * @brief Reset current animation
   *
   * @exception std::runtime_error if segment animation is not defined
   */
  void ResetAnimation()
  {
    if (!animation_) {
      throw std::runtime_error("Segment animation not defined.");
    }

    animation_->Reset();
    animation_finished_ = false;
  }

  bool HasAnimation() const { return static_cast<bool>(animation_); }

protected:
  std::shared_ptr<husarion_ugv_lights::Animation> animation_;

  const float controller_frequency_;
  bool invert_led_order_ = false;
  bool animation_finished_ = true;
  std::size_t num_led_;

  std::shared_ptr<pluginlib::ClassLoader<husarion_ugv_lights::Animation>> animation_loader_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_LAYER_INTERFACE_HPP_
