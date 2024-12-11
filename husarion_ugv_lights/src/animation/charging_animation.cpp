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

#include "husarion_ugv_lights/animation/charging_animation.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

namespace husarion_ugv_lights
{

void ChargingAnimation::Initialize(
  const YAML::Node & animation_description, const std::size_t num_led,
  const float controller_frequency)
{
  Animation::Initialize(animation_description, num_led, controller_frequency);

  fade_duration_ = int(std::round(this->GetAnimationLength() * kFadeFactor));
}

void ChargingAnimation::SetParam(const std::string & param)
{
  float battery_percent;
  try {
    battery_percent = std::clamp(std::stof(param), 0.0f, 1.0f);
  } catch (const std::invalid_argument & /*e*/) {
    throw std::runtime_error("Can not cast param to float!");
  }

  const auto anim_len = this->GetAnimationLength();
  std::size_t on_duration = static_cast<std::size_t>(std::round(float(anim_len) * battery_percent));

  on_duration = on_duration < 2 * fade_duration_ ? 2 * fade_duration_ : on_duration;

  if (on_duration >= anim_len) {
    fade_duration_ = 0;
  }

  fill_start_ = (anim_len - on_duration) / 2;
  fill_end_ = (anim_len + on_duration) / 2;
  const float hue = (kHMax - kHMin) * battery_percent + kHMin;
  color_ = HSVtoRGB(hue / 360.0, 1.0, 1.0);
}

std::vector<uint8_t> ChargingAnimation::UpdateFrame()
{
  auto anim_iteration = this->GetAnimationIteration();
  auto empty_frame = std::vector<std::uint8_t>(this->GetNumberOfLeds() * 4, 0);
  for (std::size_t i = 3; i < empty_frame.size(); i += 4) {
    empty_frame[i] = 255;
  }

  if (anim_iteration < fill_start_) {
    return empty_frame;
  }

  if (anim_iteration < fill_start_ + fade_duration_) {
    auto brightness = std::sin(M_PI_2 * (float(anim_iteration - fill_start_) / fade_duration_));
    return CreateRGBAFrame(color_, brightness);
  }

  if (anim_iteration <= fill_end_ - fade_duration_) {
    return CreateRGBAFrame(color_, 1.0);
  }

  if (anim_iteration < fill_end_) {
    auto brightness = std::sin(
      M_PI / (2.0 * fade_duration_) *
      ((float(anim_iteration) - float(fill_end_)) + 2.0 * fade_duration_));
    return CreateRGBAFrame(color_, brightness);
  }

  return empty_frame;
}

std::array<std::uint8_t, 3> ChargingAnimation::HSVtoRGB(
  const float h, const float s, const float v) const
{
  if (std::fabs(s) < std::numeric_limits<float>::epsilon()) {
    return {
      {static_cast<std::uint8_t>(v * 255), static_cast<std::uint8_t>(v * 255),
       static_cast<std::uint8_t>(v * 255)}};
  }

  const int i = static_cast<int>(std::floor(h * 6.0f));
  const float f = (h * 6.0f) - i;
  const float p = v * (1.0f - s);
  const float q = v * (1.0f - s * f);
  const float t = v * (1.0f - s * (1.0f - f));

  switch (i % 6) {
    case 0:
      return {
        {static_cast<std::uint8_t>(v * 255), static_cast<std::uint8_t>(t * 255),
         static_cast<std::uint8_t>(p * 255)}};
    case 1:
      return {
        {static_cast<std::uint8_t>(q * 255), static_cast<std::uint8_t>(v * 255),
         static_cast<std::uint8_t>(p * 255)}};
    case 2:
      return {
        {static_cast<std::uint8_t>(p * 255), static_cast<std::uint8_t>(v * 255),
         static_cast<std::uint8_t>(t * 255)}};
    case 3:
      return {
        {static_cast<std::uint8_t>(p * 255), static_cast<std::uint8_t>(q * 255),
         static_cast<std::uint8_t>(v * 255)}};
    case 4:
      return {
        {static_cast<std::uint8_t>(t * 255), static_cast<std::uint8_t>(p * 255),
         static_cast<std::uint8_t>(v * 255)}};
    default:
      return {
        {static_cast<std::uint8_t>(v * 255), static_cast<std::uint8_t>(p * 255),
         static_cast<std::uint8_t>(q * 255)}};
  }
}

std::vector<std::uint8_t> ChargingAnimation::CreateRGBAFrame(
  const std::array<std::uint8_t, 3> & color, const float brightness) const
{
  const std::array<std::uint8_t, 4> rgba_color = {
    static_cast<std::uint8_t>(color[0] * brightness),
    static_cast<std::uint8_t>(color[1] * brightness),
    static_cast<std::uint8_t>(color[2] * brightness),
    255,
  };
  std::vector<std::uint8_t> frame(this->GetNumberOfLeds() * rgba_color.size());

  for (std::size_t i = 0; i < frame.size(); i += rgba_color.size()) {
    std::copy(rgba_color.begin(), rgba_color.end(), frame.begin() + i);
  }

  return frame;
}

}  // namespace husarion_ugv_lights

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(husarion_ugv_lights::ChargingAnimation, husarion_ugv_lights::Animation)
