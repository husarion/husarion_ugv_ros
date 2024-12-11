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

#ifndef HUSARION_UGV_LIGHTS_ANIMATION_CHARGING_ANIMATION_HPP_
#define HUSARION_UGV_LIGHTS_ANIMATION_CHARGING_ANIMATION_HPP_

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/animation/animation.hpp"

namespace husarion_ugv_lights
{

class ChargingAnimation : public Animation
{
public:
  ChargingAnimation() {}
  ~ChargingAnimation() {}

  void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency) override;

  void SetParam(const std::string & param) override;

protected:
  std::vector<std::uint8_t> UpdateFrame() override;

  std::array<std::uint8_t, 3> HSVtoRGB(const float h, const float s, const float v) const;
  std::vector<std::uint8_t> CreateRGBAFrame(
    const std::array<std::uint8_t, 3> & color, const float brightness) const;

private:
  static constexpr float kFadeFactor = 0.15;
  static constexpr float kHMin = 0.0;
  static constexpr float kHMax = 120.0;

  std::size_t fade_duration_;
  std::size_t fill_start_;
  std::size_t fill_end_;
  std::array<std::uint8_t, 3> color_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_ANIMATION_CHARGING_ANIMATION_HPP_
