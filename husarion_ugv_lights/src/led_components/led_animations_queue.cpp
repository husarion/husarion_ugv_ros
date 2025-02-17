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

#include "husarion_ugv_lights/led_components/led_animations_queue.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace husarion_ugv_lights
{

LEDAnimation::LEDAnimation(
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

}  // namespace husarion_ugv_lights
