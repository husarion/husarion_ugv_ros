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

#ifndef HUSARION_UGV_LIGHTS_ANIMATION_ANIMATION_HPP_
#define HUSARION_UGV_LIGHTS_ANIMATION_ANIMATION_HPP_

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

class Animation
{
public:
  virtual ~Animation() {}

  /**
   * @brief Initialize and verify if animation was correctly defined
   *
   * @param animation_description YAML description of animation
   * @param num_led number of LEDs
   * @param controller_frequency frequency at which animation will be updated
   *
   * @exception std::out_of_range or std::runtime_error if animation
   * parameters defined in description are missing or are incorrect
   */
  virtual void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency)
  {
    Reset();
    num_led_ = num_led;
    frame_ = std::vector<std::uint8_t>(num_led_ * kRGBAColorLen, 0);

    auto duration = husarion_ugv_utils::GetYAMLKeyValue<float>(animation_description, "duration");
    if ((duration - std::numeric_limits<float>::epsilon()) <= 0.0) {
      throw std::out_of_range("Duration has to be positive");
    }

    loops_ = husarion_ugv_utils::GetYAMLKeyValue<std::size_t>(animation_description, "repeat", 1);

    if (duration * loops_ > 10.0) {
      throw std::runtime_error("Animation display duration (duration * repeat) exceeds 10 seconds");
    }

    anim_len_ = int(round(duration * controller_frequency));
    full_anim_len_ = anim_len_ * loops_;

    if (anim_len_ < 1) {
      throw std::runtime_error(
        "Animation duration is too short to display with the current frequency");
    }
  }

  /**
   * @brief Update animation frame
   *
   * @exception std::runtime_error if UpdateFrame() method returns frame with invalid size
   */
  void Update()
  {
    if (current_cycle_ >= loops_) {
      std::fill(frame_.begin(), frame_.end(), 0);
      return;
    }

    auto frame = UpdateFrame();

    if (frame.size() != num_led_ * kRGBAColorLen) {
      throw std::runtime_error(
        "Invalid frame size. Check animation UpdateFrame() method implementation");
    }

    anim_iteration_++;
    progress_ = float(anim_iteration_ + anim_len_ * current_cycle_) / full_anim_len_;

    if (anim_iteration_ >= anim_len_) {
      anim_iteration_ = 0;
      current_cycle_++;
    }

    if (current_cycle_ >= loops_) {
      finished_ = true;
    }

    frame_ = frame;
  }

  /**
   * @brief Reset animation, this will cause animation
   * to start from the beginning if Update() method is invoked
   */
  void Reset()
  {
    anim_iteration_ = 0;
    current_cycle_ = 0;
    finished_ = false;
    progress_ = 0.0;
    std::fill(frame_.begin(), frame_.end(), 0);
  }

  /**
   * @brief Return animation frame
   *
   * @param invert_frame_order if true will return frame with inverted RGBA values order (last 4
   * values will become first etc.)
   *
   * @return the newest animation frame, if animation is finished will return vector filled with 0
   */
  std::vector<std::uint8_t> GetFrame(const bool invert_frame_order = false)
  {
    return invert_frame_order ? InvertRGBAFrame(frame_) : frame_;
  }

  virtual void SetParam(const std::string & /*param*/) {};

  bool IsFinished() const { return finished_; }
  std::size_t GetNumberOfLeds() const { return num_led_; }
  float GetProgress() const { return progress_; }

  static constexpr std::size_t kRGBAColorLen = 4;

protected:
  Animation() {}

  /**
   * @brief Abstract method that has to be implemented inside child class
   * it should return RGBA animation frame with size equal to num_led_ * kRGBAColorLen
   */
  virtual std::vector<std::uint8_t> UpdateFrame() = 0;

  std::vector<std::uint8_t> InvertRGBAFrame(const std::vector<std::uint8_t> & frame) const
  {
    std::vector<std::uint8_t> inverted_frame(frame.size());
    for (std::size_t i = 0; i < frame.size(); i += kRGBAColorLen) {
      std::copy(frame.end() - i - kRGBAColorLen, frame.end() - i, inverted_frame.begin() + i);
    }
    return inverted_frame;
  }

  std::size_t GetAnimationLength() const { return anim_len_; }
  std::size_t GetAnimationIteration() const { return anim_iteration_; }

  std::vector<std::uint8_t> frame_;

private:
  std::size_t num_led_;
  std::size_t anim_len_;
  std::size_t full_anim_len_;

  bool finished_ = false;
  float progress_ = 0.0;
  std::size_t loops_;
  std::size_t current_cycle_ = 0;
  std::size_t anim_iteration_ = 0;

  std::string param_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_ANIMATION_ANIMATION_HPP_
