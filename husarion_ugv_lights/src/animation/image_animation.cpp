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

#include "husarion_ugv_lights/animation/image_animation.hpp"

#include <algorithm>
#include <cstdint>
#include <filesystem>
#include <regex>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"
#include "boost/gil/extension/numeric/resample.hpp"
#include "boost/gil/extension/numeric/sampler.hpp"

#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

void ImageAnimation::Initialize(
  const YAML::Node & animation_description, const std::size_t num_led,
  const float controller_frequency)
{
  Animation::Initialize(animation_description, num_led, controller_frequency);

  const auto image_path = ParseImagePath(
    husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation_description, "image"));
  gil::rgba8_image_t base_image;
  gil::read_and_convert_image(std::string(image_path), base_image, gil::png_tag());
  image_ = RGBAImageResize(base_image, this->GetNumberOfLeds(), this->GetAnimationLength());

  if (animation_description["color"]) {
    RGBAImageConvertColor(image_, animation_description["color"].as<std::uint32_t>());
  }
}

std::vector<std::uint8_t> ImageAnimation::UpdateFrame()
{
  std::vector<std::uint8_t> frame;
  for (std::size_t i = 0; i < this->GetNumberOfLeds(); i++) {
    auto pixel = gil::const_view(image_)(i, this->GetAnimationIteration());
    frame.push_back(pixel[0]);
    frame.push_back(pixel[1]);
    frame.push_back(pixel[2]);
    frame.push_back(pixel[3]);
  }

  return frame;
}

std::filesystem::path ImageAnimation::ParseImagePath(const std::string & image_path) const
{
  std::filesystem::path global_img_path;

  if (!std::filesystem::path(image_path).is_absolute()) {
    if (image_path[0] != '$') {
      throw std::runtime_error("Invalid image path");
    }

    std::smatch match;
    if (!std::regex_search(image_path, match, std::regex("^\\$\\(find .*\\)"))) {
      throw std::runtime_error("Can't process substitution expression");
    }

    const std::string ros_pkg_expr = match[0];
    const std::string ros_pkg = std::regex_replace(
      ros_pkg_expr, std::regex("^\\$\\(find \\s*|\\)$"), "");
    const std::string img_relative_path = image_path.substr(ros_pkg_expr.length());

    try {
      global_img_path = ament_index_cpp::get_package_share_directory(ros_pkg) + img_relative_path;
    } catch (const ament_index_cpp::PackageNotFoundError & e) {
      throw std::runtime_error("Can't find ROS package: " + ros_pkg);
    }
  } else {
    global_img_path = image_path;
  }

  if (!std::filesystem::exists(global_img_path)) {
    throw std::runtime_error("File doesn't exists: " + std::string(global_img_path));
  }

  return global_img_path;
}

gil::rgba8_image_t ImageAnimation::RGBAImageResize(
  const gil::rgba8_image_t & image, const std::size_t width, const std::size_t height) const
{
  gil::rgba8_image_t resized_image(width, height);
  gil::resize_view(gil::const_view(image), view(resized_image), gil::bilinear_sampler());

  return resized_image;
}

void ImageAnimation::RGBAImageConvertColor(
  gil::rgba8_image_t & image, const std::uint32_t color) const
{
  auto grey_image = RGBAImageConvertToGrey(image);
  GreyImageNormalizeBrightness(grey_image);

  // extract RGB values from hex
  auto r = (std::uint32_t(color) >> 16) & (0xFF);
  auto g = (std::uint32_t(color) >> 8) & (0xFF);
  auto b = (std::uint32_t(color)) & (0xFF);

  gil::transform_pixels(
    gil::const_view(grey_image), gil::view(image),
    [r, g, b](const gil::gray_alpha8_pixel_t & pixel) {
      return gil::rgba8_pixel_t(
        static_cast<std::uint8_t>(pixel[0] * r / 255),
        static_cast<std::uint8_t>(pixel[0] * g / 255),
        static_cast<std::uint8_t>(pixel[0] * b / 255), pixel[1]);
    });
}

gil::gray_alpha8_image_t ImageAnimation::RGBAImageConvertToGrey(
  const gil::rgba8_image_t & image) const
{
  gil::gray_alpha8_image_t grey_image(image.dimensions());
  gil::transform_pixels(
    gil::const_view(image), gil::view(grey_image), [](const gil::rgba8_pixel_t & pixel) {
      return gil::gray_alpha8_pixel_t(
        static_cast<std::uint8_t>(0.299 * pixel[0] + 0.587 * pixel[1] + 0.114 * pixel[2]),
        pixel[3]);
    });
  return grey_image;
}

void ImageAnimation::GreyImageNormalizeBrightness(gil::gray_alpha8_image_t & image) const
{
  std::uint8_t max_value = *std::max_element(
    gil::nth_channel_view(gil::const_view(image), 0).begin(),
    gil::nth_channel_view(gil::const_view(image), 0).end());
  gil::transform_pixels(
    gil::const_view(image), gil::view(image), [max_value](const gil::gray_alpha8_pixel_t & pixel) {
      return gil::gray_alpha8_pixel_t(
        static_cast<std::uint8_t>(float(pixel[0]) / float(max_value) * 255), pixel[1]);
    });
}

}  // namespace husarion_ugv_lights

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(husarion_ugv_lights::ImageAnimation, husarion_ugv_lights::Animation)
