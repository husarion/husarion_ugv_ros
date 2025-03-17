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
#include "rclcpp/logging.hpp"

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
    image_center_offset_ = husarion_ugv_utils::GetYAMLKeyValue<std::int16_t>(
      animation_description, "center_offset");  // in pixels
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing center_offset in animation description, using default value 0");
    image_center_offset_ = 0;
  }

  try {
    image_object_width_ = husarion_ugv_utils::GetYAMLKeyValue<std::int16_t>(
      animation_description, "object_width");  // in pixels
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing object_width in animation description, using default value 0");
    image_object_width_ = 0;
  }
  try {
    float image_start_offset_time = std::clamp(
      husarion_ugv_utils::GetYAMLKeyValue<float>(animation_description, "start_offset"), -20.0f,
      20.0f);  // in seconds
    image_start_offset_ = int(round(image_start_offset_time * controller_frequency));
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing start_offset in animation description, using default value 0");
    image_start_offset_ = 0;
  }
  try {
    float splash_duration_time = std::clamp(
      husarion_ugv_utils::GetYAMLKeyValue<float>(animation_description, "splash_duration"), 0.0f,
      20.0f);  // in seconds
    splash_duration_ = int(round(splash_duration_time * controller_frequency));
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing splash_duration in animation description, using image height");
    splash_duration_ = 0;
  }
  try {
    image_mirrored_ = husarion_ugv_utils::GetYAMLKeyValue<bool>(
      animation_description, "image_mirrored");
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing image_mirrored in animation description, assuming false");
    image_mirrored_ = false;
  }
  try {
    position_mirrored_ = husarion_ugv_utils::GetYAMLKeyValue<bool>(
      animation_description, "position_mirrored");
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing position_mirrored in animation description, assuming false");
    position_mirrored_ = false;
  }
  try {
    default_image_position_ = std::clamp(
      husarion_ugv_utils::GetYAMLKeyValue<float>(animation_description, "default_image_position"),
      0.0f, 1.0f);
    default_image_position_set_ = true;
  } catch (const std::runtime_error & /*e*/) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"),
      "Missing default_image_position in animation description, will throw if no param is "
      "provided");
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
  if (default_image_position_set_ && param.empty()) {
    image_position_ = default_image_position_;
    if (position_mirrored_) {
      image_position_ = 1.0f - image_position_;
    }
    RCLCPP_DEBUG(
      rclcpp::get_logger("MovingImageAnimation"), "Setting image position to default: %f",
      default_image_position_);
    return;
  }

  try {
    image_position_ = std::clamp(std::stof(param), 0.0f, 1.0f);
    if (position_mirrored_) {
      image_position_ = 1.0f - image_position_;
    }
  } catch (const std::invalid_argument & /*e*/) {
    throw std::runtime_error("Can not cast param to float!");
  }
}

std::vector<std::uint8_t> MovingImageAnimation::UpdateFrame()
{
  int16_t left_edge_position;
  if (image_mirrored_) {
    left_edge_position =
      static_cast<int>(
        image_position_ * static_cast<int>(GetNumberOfLeds() - (image_object_width_))) -
      (image_.width() - image_center_offset_ - image_object_width_);
  } else {
    left_edge_position =
      static_cast<int>(
        image_position_ * static_cast<int>(GetNumberOfLeds() - (image_object_width_))) -
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

}  // namespace husarion_ugv_lights

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(husarion_ugv_lights::MovingImageAnimation, husarion_ugv_lights::Animation)
