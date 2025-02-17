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

#ifndef HUSARION_UGV_LIGHTS_MOVING_ANIMATION_IMAGE_ANIMATION_HPP_
#define HUSARION_UGV_LIGHTS_MOVING_ANIMATION_IMAGE_ANIMATION_HPP_

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "husarion_ugv_lights/animation/image_animation.hpp"

namespace gil = boost::gil;

namespace husarion_ugv_lights
{

class MovingImageAnimation : public ImageAnimation
{
public:
  MovingImageAnimation() {}
  ~MovingImageAnimation() {}

  void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency);

protected:
  std::vector<std::uint8_t> UpdateFrame();

  void SetParam(const std::string & param);

private:
  float image_position_;
  float default_image_position_;
  bool default_image_position_set_;
  bool image_mirrored_;
  bool position_mirrored_;
  std::size_t image_center_offset_;
  std::size_t image_object_width_;
  std::int32_t image_start_offset_;
  std::size_t splash_duration_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_MOVING_ANIMATION_IMAGE_ANIMATION_HPP_
