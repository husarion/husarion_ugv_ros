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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATIONS_QUEUE_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATIONS_QUEUE_HPP_

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
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
    const rclcpp::Time & init_time);

  ~LEDAnimation() {}

  /**
   * @brief Indicates if LED animation is finished
   *
   * @return True if all animations on all segments are finished, false otherwise
   */
  bool IsFinished();

  /**
   * @brief Reset all animations on all LED segments
   *
   * @param time This time is used to set new animation initialization time
   */
  void Reset(const rclcpp::Time & time);

  std::string GetName() const { return led_animation_description_.name; }
  std::uint8_t GetPriority() const { return led_animation_description_.priority; }
  std::vector<AnimationDescription> GetAnimations() const
  {
    return led_animation_description_.animations;
  }
  rclcpp::Time GetInitTime() const { return init_time_; }
  float GetTimeout() const { return led_animation_description_.timeout; }

  /**
   * @brief Get LED animation progress
   *
   * @return The smallest progress of all animations on all segments
   */
  float GetProgress() const;

  bool IsRepeating() const { return repeating_; }
  std::string GetParam() const { return param_; }

  void SetRepeating(const bool value) { repeating_ = value; }
  void SetParam(const std::string & param) { param_ = param; }

  static constexpr std::uint8_t kDefaultPriority = 3;
  static constexpr float kDefaultTimeout = 120.0f;
  static constexpr std::array<std::uint8_t, 3> kValidPriorities = {1, 2, 3};

private:
  const LEDAnimationDescription led_animation_description_;
  rclcpp::Time init_time_;

  bool repeating_;
  std::string param_;
  std::vector<std::shared_ptr<LEDSegment>> animation_segments_;
};

/**
 * @brief Class used to manage queue of LED animations
 */
class LEDAnimationsQueue
{
public:
  /**
   * @brief Initializes LED animations queue
   *
   * @param max_queue_size Max size of the queue
   */
  LEDAnimationsQueue(const std::size_t max_queue_size = 5) : max_queue_size_(max_queue_size) {}

  /**
   * @brief Add animation to the queue and sort animations according to their priority and time of
   * initialization
   *
   * @param animation LED animation to add to queue
   * @param time Time of initialization of the animation
   *
   * @exception std::runtime_error if queue has number of elements equal to max_queue_size
   */
  void Put(const std::shared_ptr<LEDAnimation> & animation, const rclcpp::Time & time);

  /**
   * @brief Get and remove first LED animation from the queue
   *
   * @return First LED animation from the queue
   *
   * @exception std::runtime_error if queue is empty
   */
  std::shared_ptr<LEDAnimation> Get();

  /**
   * @brief Remove all animations with priority equal or lower to specified one
   *
   * @param priority Animation with this priority or lower will be removed from the queue
   */
  void Clear(const std::size_t priority = 2);

  /**
   * @brief Removes animations that has reached theirs timeout from the queue
   *
   * @param time Time used to check if animation has timed out
   */
  void Validate(const rclcpp::Time & time);

  /**
   * @brief Return priority of the first animation in the queue
   *
   * @return Priority of the first animation or default animation priority if queue is empty
   */
  std::size_t GetFirstAnimationPriority() const;

  void Remove(const std::shared_ptr<LEDAnimation> & animation);
  bool HasAnimation(const std::shared_ptr<LEDAnimation> & animation) const;

  bool Empty() const { return queue_.empty(); }

private:
  std::deque<std::shared_ptr<LEDAnimation>> queue_;
  const std::size_t max_queue_size_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_ANIMATIONS_QUEUE_HPP_
