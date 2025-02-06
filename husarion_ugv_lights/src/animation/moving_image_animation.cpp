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

#include "husarion_ugv_lights/animation/moving_image_animation.hpp"

#include <algorithm>
#include <cstddef>
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
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

void MovingImageAnimation::Initialize(
  const YAML::Node & animation_description, const std::size_t num_led,
  const float controller_frequency)
{
  Animation::Initialize(animation_description, num_led, controller_frequency);

  const auto image_path = ParseImagePath(
    husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation_description, "image"));

  try {
    image_center_offset_ =
      static_cast<int16_t>(std::stoi(husarion_ugv_utils::GetYAMLKeyValue<std::string>(
        animation_description, "center_offset")));  // in pixels
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing center_offset in animation description, using default value 0");
    image_center_offset_ = 0;
  }

  try {
    image_object_width_ =
      static_cast<int16_t>(std::stoi(husarion_ugv_utils::GetYAMLKeyValue<std::string>(
        animation_description, "object_width")));  // in pixels
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing object_width in animation description, using default value 0");
    image_object_width_ = 0;
  }
  try {
    float image_start_offset_time = std::clamp(
      std::stof(
        husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation_description, "start_offset")),
      -20.0f, 20.0f);  // in seconds
    image_start_offset_ = int(round(image_start_offset_time * controller_frequency));
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing start_offset in animation description, using default value 0");
    image_start_offset_ = 0;
  }
  try {
    float splash_duration_time = std::clamp(
      std::stof(
        husarion_ugv_utils::GetYAMLKeyValue<std::string>(animation_description, "splash_duration")),
      0.0f, 20.0f);  // in seconds
    splash_duration_ = int(round(splash_duration_time * controller_frequency));
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing start_offset in animation description, using image height");
    splash_duration_ = 0;
  }
  try {
    image_mirrored_ = husarion_ugv_utils::GetYAMLKeyValue<bool>(
      animation_description, "image_mirrored");
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing image_mirrored in animation description, assuming false");
    image_mirrored_ = false;
  }
  try {
    position_mirrored_ = husarion_ugv_utils::GetYAMLKeyValue<bool>(
      animation_description, "position_mirrored");
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_INFO(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing position_mirrored in animation description, assuming false");
    position_mirrored_ = false;
  }

  gil::rgba8_image_t base_image;
  gil::read_and_convert_image(std::string(image_path), base_image, gil::png_tag());
  if (splash_duration_ > 0) {
    image_ = RGBAImageResize(base_image, base_image.width(), splash_duration_);
  } else {
    splash_duration_ = base_image.height();
    image_ = base_image;
  }
  if (animation_description["color"]) {
    RGBAImageConvertColor(image_, animation_description["color"].as<std::uint32_t>());
  }
}

void MovingImageAnimation::SetParam(const std::string & param)
{
  // try {
  //   image_position_ = std::stof(param);
  // } catch (const std::invalid_argument & /*e*/) {
  //   throw std::runtime_error("Can not cast param to int!");
  // }

  try {
    image_position_ = std::clamp(std::stof(param), 0.0f, 1.0f);
    if (position_mirrored_) {
      image_position_ = 1.0f - image_position_;
    }
  } catch (const std::invalid_argument & /*e*/) {
    throw std::runtime_error("Can not cast param to float!");
  }

  // const auto anim_len = this->GetAnimationLength();
  // std::size_t on_duration = static_cast<std::size_t>(std::round(float(anim_len) *
  // battery_percent));

  // on_duration = on_duration < 2 * fade_duration_ ? 2 * fade_duration_ : on_duration;

  // if (on_duration >= anim_len) {
  //   fade_duration_ = 0;
  // }

  // fill_start_ = (anim_len - on_duration) / 2;
  // fill_end_ = (anim_len + on_duration) / 2;
  // const float hue = (kHMax - kHMin) * battery_percent + kHMin;
  // color_ = HSVtoRGB(hue / 360.0, 1.0, 1.0);
}

std::vector<std::uint8_t> MovingImageAnimation::UpdateFrame()
{
  int16_t left_edge_position;
  if (image_mirrored_) {
    left_edge_position = (int)(image_position_ * (int)(GetNumberOfLeds() - (image_object_width_))) -
                         (image_.width() - image_center_offset_ - image_object_width_);
  } else {
    left_edge_position = (int)(image_position_ * (int)(GetNumberOfLeds() - (image_object_width_))) -
                         image_center_offset_;
  }
  int16_t right_edge_position = left_edge_position + (image_.width());
  int16_t top_edge_position = image_start_offset_;
  int16_t bottom_edge_position = top_edge_position + splash_duration_;

  size_t left_range = std::clamp(
    left_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(GetNumberOfLeds()));
  size_t right_range = std::clamp(
    right_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(GetNumberOfLeds()));
  size_t top_range = std::clamp(
    top_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(GetAnimationLength()));
  size_t bottom_range = std::clamp(
    bottom_edge_position, static_cast<int16_t>(0), static_cast<int16_t>(GetAnimationLength()));

  std::vector<std::uint8_t> frame;
  for (std::size_t i = 0; i < GetNumberOfLeds(); i++) {
    if (
      i >= left_range && i < right_range && GetAnimationIteration() >= top_range &&
      GetAnimationIteration() < bottom_range) {
      size_t pixel_index;
      if (image_mirrored_) {
        pixel_index = image_.width() - (i - left_edge_position) - 1;
      } else {
        pixel_index = i - left_edge_position;
      }

      auto pixel = gil::const_view(image_)(
        pixel_index, GetAnimationIteration() - top_edge_position);
      frame.push_back(pixel[0]);
      frame.push_back(pixel[1]);
      frame.push_back(pixel[2]);
      frame.push_back(pixel[3]);
    } else {
      frame.push_back(0);
      frame.push_back(0);
      frame.push_back(0);
      frame.push_back(0);
    }
  }

  return frame;
}

std::filesystem::path MovingImageAnimation::ParseImagePath(const std::string & image_path) const
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

gil::rgba8_image_t MovingImageAnimation::RGBAImageResize(
  const gil::rgba8_image_t & image, const std::size_t width, const std::size_t height) const
{
  gil::rgba8_image_t resized_image(width, height);
  gil::resize_view(gil::const_view(image), view(resized_image), gil::bilinear_sampler());

  return resized_image;
}

void MovingImageAnimation::RGBAImageConvertColor(
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

gil::gray_alpha8_image_t MovingImageAnimation::RGBAImageConvertToGrey(
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

void MovingImageAnimation::GreyImageNormalizeBrightness(gil::gray_alpha8_image_t & image) const
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

PLUGINLIB_EXPORT_CLASS(husarion_ugv_lights::MovingImageAnimation, husarion_ugv_lights::Animation)
