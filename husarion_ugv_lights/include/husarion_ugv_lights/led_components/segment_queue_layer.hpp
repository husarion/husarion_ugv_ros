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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "pluginlib/class_loader.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"
#include "husarion_ugv_lights/led_components/segment_layer_interface.hpp"

namespace husarion_ugv_lights
{

/**
 * @brief Class that represents virtual LED segment of the robot
 */
class SegmentQueueLayer : public SegmentLayerInterface
{
public:
  /**
   * @brief Parses basic parameters of the LED segment
   *
   * @param num_led number of LEDs
   * @param invert_led_order if true will invert the order of LEDs
   * @param controller_frequency frequency at which animation will be updated.
   *
   * @exception std::runtime_error or std::invalid_argument if missing required description key or
   * key couldn't be parsed
   */
  SegmentQueueLayer(
    const std::size_t num_led, const bool invert_led_order, const float controller_frequency);

  /**
   * @brief Overwrite current animation
   *
   * @param type pluginlib animation type
   * @param animation_description YAML description of the animation. Must contain 'type' key -
   * pluginlib animation type
   * @param repeating if true, will set the default animation for the panel
   * @param param optional parameter to pass to animation when initializing
   *
   * @exception std::runtime_error if 'type' key is missing, given pluginlib fails to load or
   * animation fails to initialize
   */
  void SetAnimation(
    const std::string & type, const YAML::Node & animation_description, const bool repeating,
    const std::string & param = "") override;

  /**
   * @brief Update animation frame
   *
   * @exception std::runtime_error if fails to update animation
   */
  void UpdateAnimation() override;

protected:
  std::deque<std::shared_ptr<husarion_ugv_lights::Animation>> animations_queue_;
  std::size_t max_queue_size_ = 10;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_QUEUE_LAYER_HPP_
