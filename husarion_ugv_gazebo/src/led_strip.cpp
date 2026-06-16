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

#include "husarion_ugv_gazebo/led_strip.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <sstream>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/LightCmd.hh>
#include <gz/sim/components/Name.hh>

#include <gz/msgs/color.pb.h>
#include <gz/msgs/marker.pb.h>

namespace husarion_ugv_gazebo
{
void LEDStrip::Configure(
  const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & /*eventMgr*/)
{
  const auto model = gz::sim::Model(entity);
  if (!model.Valid(ecm)) {
    throw std::runtime_error(
      "Error: Failed to initialize because [" + model.Name(ecm) +
      "] (Entity=" + std::to_string(entity) +
      ") is not a model. Please make sure that LEDStrip is attached to a valid model.");
  }

  model_name_ = model.Name(ecm);

  ParseParameters(sdf);
  light_cmd_ = SetupLightCmd(ecm);

  node_.Subscribe(ns_ + "/" + image_topic_, &LEDStrip::ImageCallback, this);
}

void LEDStrip::PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
{
  auto current_time = info.simTime;
  auto period = std::chrono::milliseconds(static_cast<int>(1000 / frequency_));

  if (new_image_available_ && current_time - last_update_time_ >= period) {
    gz::msgs::Image image;
    {
      last_image_.get(image);
      new_image_available_ = false;
    }
    last_update_time_ = current_time;

    if (image.data() == last_rendered_data_) {
      return;
    }
    last_rendered_data_ = image.data();

    VisualizeLights(ecm, image);

    auto light_pose = ecm.Component<gz::sim::components::Pose>(light_entity_)->Data();
    VisualizeMarkers(image, light_pose);
  }
}

void LEDStrip::ParseParameters(const std::shared_ptr<const sdf::Element> & sdf)
{
  if (!sdf->HasElement("light_name")) {
    throw std::runtime_error("Error: The light_name parameter is missing.");
  }
  light_name_ = sdf->Get<std::string>("light_name");

  if (!sdf->HasElement("topic")) {
    throw std::runtime_error("Error: The topic parameter is missing.");
  }
  image_topic_ = sdf->Get<std::string>("topic");

  ns_ = sdf->HasElement("namespace") ? sdf->Get<std::string>("namespace") : ns_;
  frequency_ = sdf->HasElement("frequency") ? sdf->Get<double>("frequency") : frequency_;
  marker_width_ = sdf->HasElement("width") ? sdf->Get<double>("width") : marker_width_;
  marker_height_ = sdf->HasElement("height") ? sdf->Get<double>("height") : marker_height_;

  blur_sigma_ = sdf->HasElement("blur_sigma") ? sdf->Get<double>("blur_sigma") : blur_sigma_;

  if (sdf->HasElement("led_range")) {
    const auto range = sdf->Get<std::string>("led_range");
    const auto dash = range.find('-');
    if (dash == std::string::npos) {
      throw std::runtime_error("Error: led_range must have the <first>-<last> format.");
    }
    led_first_ = std::stoi(range.substr(0, dash));
    led_last_ = std::stoi(range.substr(dash + 1));
  }

  if (sdf->HasElement("points")) {
    std::istringstream points_stream(sdf->Get<std::string>("points"));
    std::string point_token;
    while (std::getline(points_stream, point_token, ';')) {
      std::istringstream point(point_token);
      double x, y, z;
      if (point >> x >> y >> z) {
        points_.emplace_back(x, y, z);
      }
    }
    if (points_.size() < 2) {
      throw std::runtime_error("Error: The points parameter requires at least 2 'x y z' points.");
    }
    for (size_t i = 0; i + 1 < points_.size(); ++i) {
      const double length = (points_[i + 1] - points_[i]).Length();
      polyline_seg_lengths_.push_back(length);
      polyline_length_ += length;
    }
    if (polyline_length_ <= 0.0) {
      throw std::runtime_error("Error: The points parameter defines a zero-length polyline.");
    }
  }
}

gz::msgs::Light LEDStrip::SetupLightCmd(gz::sim::EntityComponentManager & ecm)
{
  gz::msgs::Light light_cmd;

  ecm.Each<gz::sim::components::Name, gz::sim::components::Light>(
    [&](
      const gz::sim::Entity & entity, const gz::sim::components::Name * name,
      const gz::sim::components::Light * light_component) -> bool {
      if (name->Data() != light_name_) {
        return true;  // Continue searching
      }
      light_entity_ = entity;
      gzdbg << "Light entity found: " << light_entity_ << std::endl;
      light_cmd = CreateLightMsgFromSdf(light_component->Data());
      return false;  // Stop searching
    });

  if (light_entity_ == gz::sim::kNullEntity) {
    throw std::runtime_error("Error: Light entity not found.");
  }
  return light_cmd;
}

gz::msgs::Light LEDStrip::CreateLightMsgFromSdf(const sdf::Light & light_sdf)
{
  gz::msgs::Light light_cmd;

  light_cmd.set_name(light_sdf.Name());
  light_cmd.set_range(light_sdf.AttenuationRange());
  light_cmd.set_cast_shadows(light_sdf.CastShadows());
  light_cmd.set_spot_inner_angle(light_sdf.SpotInnerAngle().Radian());
  light_cmd.set_spot_outer_angle(light_sdf.SpotOuterAngle().Radian());
  light_cmd.set_spot_falloff(light_sdf.SpotFalloff());
  light_cmd.set_attenuation_constant(light_sdf.ConstantAttenuationFactor());
  light_cmd.set_attenuation_linear(light_sdf.LinearAttenuationFactor());
  light_cmd.set_attenuation_quadratic(light_sdf.QuadraticAttenuationFactor());
  light_cmd.set_intensity(light_sdf.Intensity());

  gz::msgs::Set(light_cmd.mutable_diffuse(), light_sdf.Diffuse());
  gz::msgs::Set(light_cmd.mutable_specular(), light_sdf.Specular());
  gz::msgs::Set(light_cmd.mutable_direction(), light_sdf.Direction());

  // Set the light type
  switch (light_sdf.Type()) {
    case sdf::LightType::POINT:
      light_cmd.set_type(gz::msgs::Light::POINT);
      break;
    case sdf::LightType::SPOT:
      light_cmd.set_type(gz::msgs::Light::SPOT);
      break;
    case sdf::LightType::DIRECTIONAL:
      light_cmd.set_type(gz::msgs::Light::DIRECTIONAL);
      break;
    case sdf::LightType::INVALID:
    default:
      light_cmd.set_type(gz::msgs::Light::POINT);
      break;
  }

  return light_cmd;
}

void LEDStrip::ImageCallback(const gz::msgs::Image & msg)
{
  if (!IsEncodingValid(msg)) {
    gzerr << "Error: Incorrect image encoding." << std::endl;
    return;
  }

  const size_t step = msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8 ? 4 : 3;
  if (msg.data().size() < msg.width() * msg.height() * step) {
    gzerr << "Error: Image data smaller than the declared dimensions." << std::endl;
    return;
  }

  last_image_.set(msg);
  new_image_available_ = true;
}

bool LEDStrip::IsEncodingValid(const gz::msgs::Image & msg)
{
  return msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8 ||
         msg.pixel_format_type() == gz::msgs::PixelFormatType::RGB_INT8;
}

gz::math::Color LEDStrip::CalculateMeanColor(const gz::msgs::Image & msg)
{
  size_t sum_r = 0, sum_g = 0, sum_b = 0, sum_a = 0;
  size_t pixel_count = msg.width() * msg.height();

  const std::string & data = msg.data();
  bool is_rgba = (msg.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  int step = is_rgba ? 4 : 3;

  for (size_t i = 0; i < pixel_count * step; i += step) {
    sum_r += static_cast<uint8_t>(data[i]);
    sum_g += static_cast<uint8_t>(data[i + 1]);
    sum_b += static_cast<uint8_t>(data[i + 2]);
    if (is_rgba) {
      sum_a += static_cast<uint8_t>(data[i + 3]);
    }
  }

  const float max_value = std::numeric_limits<uint8_t>::max();
  const float norm_factor = max_value * pixel_count;

  float norm_mean_r = sum_r / norm_factor;
  float norm_mean_g = sum_g / norm_factor;
  float norm_mean_b = sum_b / norm_factor;
  float norm_mean_a = is_rgba ? sum_a / norm_factor : 1.0f;

  auto mean_color = gz::math::Color(norm_mean_r, norm_mean_g, norm_mean_b, norm_mean_a);

  return mean_color;
}

void LEDStrip::VisualizeLights(gz::sim::EntityComponentManager & ecm, const gz::msgs::Image & image)
{
  gz::math::Color mean_color = CalculateMeanColor(image);

  gz::msgs::Set(light_cmd_.mutable_diffuse(), mean_color);
  gz::msgs::Set(light_cmd_.mutable_specular(), mean_color);

  auto light_on = light_cmd_.mutable_header()->add_data();
  light_on->set_key("isLightOn");
  light_on->add_value()->assign("1");

  auto visualize = light_cmd_.mutable_header()->add_data();
  visualize->set_key("visualizeVisual");
  visualize->add_value()->assign("0");

  ecm.SetComponentData<gz::sim::components::LightCmd>(light_entity_, light_cmd_);

  ecm.SetChanged(
    light_entity_, gz::sim::components::LightCmd::typeId, gz::sim::ComponentState::PeriodicChange);
}

std::vector<gz::math::Color> LEDStrip::ExtractLedColors(const gz::msgs::Image & image) const
{
  const std::string & data = image.data();
  const float max_value = std::numeric_limits<uint8_t>::max();
  const bool is_rgba = (image.pixel_format_type() == gz::msgs::PixelFormatType::RGBA_INT8);
  const int step = is_rgba ? 4 : 3;

  const size_t pixel_count = image.width() * image.height();
  const int first = led_first_ >= 0 ? led_first_ : 0;
  const int last = led_last_ >= 0 ? led_last_ : static_cast<int>(pixel_count) - 1;
  const int dir = last >= first ? 1 : -1;
  const size_t led_count = static_cast<size_t>(std::abs(last - first)) + 1;

  std::vector<gz::math::Color> colors;
  colors.reserve(led_count);
  for (size_t i = 0; i < led_count; ++i) {
    const size_t pixel_idx = static_cast<size_t>(first + dir * static_cast<int>(i));
    if (pixel_idx >= pixel_count) {
      gzerr << "Error: led_range exceeds the received frame size." << std::endl;
      return {};
    }
    const size_t idx = pixel_idx * step;
    const float r = static_cast<uint8_t>(data[idx]) / max_value;
    const float g = static_cast<uint8_t>(data[idx + 1]) / max_value;
    const float b = static_cast<uint8_t>(data[idx + 2]) / max_value;
    const float a = is_rgba ? static_cast<uint8_t>(data[idx + 3]) / max_value : 1.0f;
    colors.emplace_back(r, g, b, a);
  }
  return colors;
}

void LEDStrip::ApplyGaussianBlur(std::vector<gz::math::Color> & colors) const
{
  if (blur_sigma_ <= 0.0 || colors.size() < 2) {
    return;
  }

  const int radius = static_cast<int>(std::ceil(3.0 * blur_sigma_));
  std::vector<double> kernel(radius + 1);
  double norm = 0.0;
  for (int k = 0; k <= radius; ++k) {
    kernel[k] = std::exp(-(k * k) / (2.0 * blur_sigma_ * blur_sigma_));
    norm += (k == 0 ? kernel[k] : 2.0 * kernel[k]);
  }

  const int n = static_cast<int>(colors.size());
  std::vector<gz::math::Color> blurred(colors.size());
  for (int i = 0; i < n; ++i) {
    double r = 0.0, g = 0.0, b = 0.0, a = 0.0;
    for (int k = -radius; k <= radius; ++k) {
      const int j = std::clamp(i + k, 0, n - 1);  // extend edges
      const double w = kernel[std::abs(k)] / norm;
      r += w * colors[j].R();
      g += w * colors[j].G();
      b += w * colors[j].B();
      a += w * colors[j].A();
    }
    blurred[i].Set(r, g, b, a);
  }
  colors = std::move(blurred);
}

void LEDStrip::VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & light_pose)
{
  auto colors = ExtractLedColors(image);
  if (colors.empty()) {
    return;
  }
  ApplyGaussianBlur(colors);

  const size_t led_count = colors.size();
  if (last_marker_colors_.size() != led_count) {
    last_marker_colors_.assign(led_count, gz::math::Color(-1.0f, -1.0f, -1.0f, -1.0f));
  }

  const double step_width = marker_width_ / led_count;

  for (size_t i = 0; i < led_count; ++i) {
    if (colors[i] == last_marker_colors_[i]) {
      continue;
    }
    last_marker_colors_[i] = colors[i];

    if (!points_.empty()) {
      CreateRibbonMarker(i, light_pose, colors[i], PolylineLedVertices(i, led_count));
    } else {
      // Legacy strip: a straight line along the Y axis at the light position.
      const auto pose = gz::math::Pose3d(
        light_pose.Pos().X(),
        light_pose.Pos().Y() + i * step_width - marker_width_ / 2.0 + step_width / 2.0,
        light_pose.Pos().Z(), light_pose.Rot().Roll(), light_pose.Rot().Pitch(),
        light_pose.Rot().Yaw());
      const auto size = gz::math::Vector3d(0.001, step_width, marker_height_);
      CreateMarker(i, pose, colors[i], size);
    }
  }
}

gz::math::Vector3d LEDStrip::PolylinePointAt(double distance) const
{
  size_t segment = 0;
  double segment_start = 0.0;
  while (segment + 1 < polyline_seg_lengths_.size() &&
         segment_start + polyline_seg_lengths_[segment] < distance) {
    segment_start += polyline_seg_lengths_[segment];
    ++segment;
  }

  const double t = std::clamp(
    (distance - segment_start) / polyline_seg_lengths_[segment], 0.0, 1.0);
  return points_[segment] + t * (points_[segment + 1] - points_[segment]);
}

std::vector<gz::math::Vector3d> LEDStrip::PolylineLedVertices(
  size_t led_idx, size_t led_count) const
{
  const double led_step = polyline_length_ / led_count;
  const double led_start = led_idx * led_step;
  const double led_end = (led_idx + 1) * led_step;

  std::vector<gz::math::Vector3d> vertices;
  vertices.push_back(PolylinePointAt(led_start));

  double vertex_distance = 0.0;
  for (size_t i = 0; i < polyline_seg_lengths_.size(); ++i) {
    vertex_distance += polyline_seg_lengths_[i];
    if (vertex_distance > led_start && vertex_distance < led_end) {
      vertices.push_back(points_[i + 1]);
    }
  }

  vertices.push_back(PolylinePointAt(led_end));
  return vertices;
}

void LEDStrip::CreateMarker(
  const uint id, const gz::math::Pose3d pose, const gz::math::Color & color,
  const gz::math::Vector3d size)
{
  gz::msgs::Marker marker_msg;
  marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_msg.set_ns(ns_ + light_name_);
  marker_msg.set_id(id + 1);  // Markers with IDs 0 cannot be overwritten
  marker_msg.set_parent(ns_.empty() ? model_name_ : ns_);
  marker_msg.set_type(gz::msgs::Marker::BOX);

  gz::msgs::Set(marker_msg.mutable_pose(), pose);
  gz::msgs::Set(marker_msg.mutable_scale(), size);

  gz::msgs::Set(marker_msg.mutable_material()->mutable_ambient(), color);
  gz::msgs::Set(marker_msg.mutable_material()->mutable_diffuse(), color);

  // Using Request to ensure markers are visible
  node_.Request("/marker", marker_msg);
}

void LEDStrip::CreateRibbonMarker(
  const uint id, const gz::math::Pose3d & strip_pose, const gz::math::Color & color,
  const std::vector<gz::math::Vector3d> & centerline)
{
  gz::msgs::Marker marker_msg;
  marker_msg.set_action(gz::msgs::Marker::ADD_MODIFY);
  marker_msg.set_ns(ns_ + light_name_);
  marker_msg.set_id(id + 1);  // Markers with IDs 0 cannot be overwritten
  marker_msg.set_parent(ns_.empty() ? model_name_ : ns_);
  marker_msg.set_type(gz::msgs::Marker::TRIANGLE_LIST);

  gz::msgs::Set(marker_msg.mutable_pose(), strip_pose);

  const auto up = gz::math::Vector3d(0.0, 0.0, marker_height_ / 2.0);
  auto add_triangle =
    [&](const gz::math::Vector3d & a, const gz::math::Vector3d & b, const gz::math::Vector3d & c) {
      gz::msgs::Set(marker_msg.add_point(), a);
      gz::msgs::Set(marker_msg.add_point(), b);
      gz::msgs::Set(marker_msg.add_point(), c);
    };

  // Two triangles per centerline section, both windings so the ribbon is visible from both sides.
  for (size_t i = 0; i + 1 < centerline.size(); ++i) {
    const auto a_bottom = centerline[i] - up;
    const auto a_top = centerline[i] + up;
    const auto b_bottom = centerline[i + 1] - up;
    const auto b_top = centerline[i + 1] + up;

    add_triangle(a_bottom, b_bottom, a_top);
    add_triangle(a_top, b_bottom, b_top);
    add_triangle(a_bottom, a_top, b_bottom);
    add_triangle(b_bottom, a_top, b_top);
  }

  gz::msgs::Set(marker_msg.mutable_material()->mutable_ambient(), color);
  gz::msgs::Set(marker_msg.mutable_material()->mutable_diffuse(), color);

  // Using Request to ensure markers are visible
  node_.Request("/marker", marker_msg);
}

}  // namespace husarion_ugv_gazebo

GZ_ADD_PLUGIN(
  husarion_ugv_gazebo::LEDStrip, gz::sim::System, husarion_ugv_gazebo::LEDStrip::ISystemConfigure,
  husarion_ugv_gazebo::LEDStrip::ISystemPreUpdate)
