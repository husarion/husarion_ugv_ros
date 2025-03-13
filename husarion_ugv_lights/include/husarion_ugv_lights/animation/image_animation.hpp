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

#ifndef HUSARION_UGV_LIGHTS_ANIMATION_IMAGE_ANIMATION_HPP_
#define HUSARION_UGV_LIGHTS_ANIMATION_IMAGE_ANIMATION_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "boost/gil.hpp"
#include "boost/gil/extension/toolbox/color_spaces/gray_alpha.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"

namespace gil = boost::gil;

namespace husarion_ugv_lights
{

class ImageAnimation : public Animation
{
public:
  ImageAnimation() {}
  ~ImageAnimation() {}

  void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency) override;

protected:
  std::vector<std::uint8_t> UpdateFrame() override;

  /**
   * @brief Process raw image path including extracting ros package shared directory path specified
   * with '$(find ros_package)` syntax
   *
   * @param image_path raw path to an image, it should be a global path or should contain '$(find
   * ros_package)` syntax
   *
   * @return global path to an image file
   * @exception std::runtime_error if provided image_path is invalid or file does not exists
   */
  std::filesystem::path ParseImagePath(const std::string & image_path) const;

  gil::rgba8_image_t RGBAImageResize(
    const gil::rgba8_image_t & image, const std::size_t width, const std::size_t height) const;

  /**
   * @brief This method converts RGB image to gray, normalizes gray image brightness and then
   * applies provided color
   *
   * @param image RGB image that will be converted
   * @param color 24-bit RGB color
   */
  void RGBAImageConvertColor(gil::rgba8_image_t & image, const std::uint32_t color) const;

  gil::gray_alpha8_image_t RGBAImageConvertToGrey(const gil::rgba8_image_t & image) const;

  void GreyImageNormalizeBrightness(gil::gray_alpha8_image_t & image) const;

protected:
  gil::rgba8_image_t image_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_ANIMATION_IMAGE_ANIMATION_HPP_
