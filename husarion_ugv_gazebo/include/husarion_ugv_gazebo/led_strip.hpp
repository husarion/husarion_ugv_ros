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

#ifndef HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_LED_STRIP_HPP_
#define HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_LED_STRIP_HPP_

#include <chrono>
#include <string>
#include <vector>

#include <realtime_tools/realtime_thread_safe_box.hpp>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/light.pb.h>
#include <gz/msgs/marker.pb.h>

namespace husarion_ugv_gazebo
{

/**
 * @brief Class to manage an LED strip in a Gazebo simulation based on received image.
 */
class LEDStrip : public gz::sim::System,
                 public gz::sim::ISystemConfigure,
                 public gz::sim::ISystemPreUpdate
{
public:
  /**
   * @brief Configures the LED strip. This function fill up parameters and light_cmd_ based on URDF.
   * Inherit from gz::sim::ISystemConfigure. More information can be found in the [Gazebo
   * documentation](https://gazebosim.org/api/gazebo/6/createsystemplugins.html).
   *
   * @param id The entity ID of the model.
   * @param sdf The SDF element of the model.
   * @param ecm The entity component manager.
   * @param eventMgr The event manager.
   *
   * @exception std::runtime_error if the entity is not a model.
   */
  void Configure(
    const gz::sim::Entity & id, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & eventMgr) override;

  /**
   * @brief Displays lights and markers, with specified by URDF frequency. Inherit from
   * gz::sim::ISystemPreUpdate. More information can be found in the [Gazebo
   * documentation](https://gazebosim.org/api/gazebo/6/createsystemplugins.html).
   *
   * @param info The update information.
   * @param ecm The entity component manager.
   */
  void PreUpdate(const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override;

private:
  /**
   * @brief Parse parameters from the URDF file
   *
   * @param sdf The SDF element of the model.
   * @exception std::runtime_error if the light_name or topic parameter is missing.
   */
  void ParseParameters(const std::shared_ptr<const sdf::Element> & sdf);

  /**
   * @brief Return Light command based on light configuration specified in URDF file Light
   * properties
   *
   * @param ecm The entity component manager.
   * @return Light command message
   * @exception std::runtime_error if the light entity is not found.
   */
  gz::msgs::Light SetupLightCmd(gz::sim::EntityComponentManager & ecm);

  /**
   * @brief Convert sdf::Light (configuration from URDF) to gz::msgs::Light (command msg)
   *
   * @param light_sdf Light SDF configuration
   * @return Light command message
   */
  gz::msgs::Light CreateLightMsgFromSdf(const sdf::Light & light_sdf);
  void ImageCallback(const gz::msgs::Image & msg);
  bool IsEncodingValid(const gz::msgs::Image & msg);
  gz::math::Color CalculateMeanColor(const gz::msgs::Image & msg);

  /**
   * @brief Manage color of robot simulated lights based on the image message
   *
   * @param ecm The entity component manager.
   * @param image The image message
   */
  void VisualizeLights(gz::sim::EntityComponentManager & ecm, const gz::msgs::Image & image);

  /**
   * @brief Manage color of robot LED strip based on the image message
   *
   * @param image The image message
   * @param light_pose The pose of the light
   */
  void VisualizeMarkers(const gz::msgs::Image & image, const gz::math::Pose3d & light_pose);

  /**
   * @brief Extract the per-LED colors of this instance's subrange from the image, applying the
   * led_range order. Returns colors ordered from the first to the last LED of the strip.
   */
  std::vector<gz::math::Color> ExtractLedColors(const gz::msgs::Image & image) const;

  /**
   * @brief Reconstruct the strip at sub-LED resolution: subdivide each LED into
   * render_markers_per_led_ cells and linearly interpolate between neighboring LED-center colors,
   * approximating the diffuser without bleeding across gaps the way a wide kernel would. Each cell
   * color is composited over strip_base_color_ so transparent animations reveal the diffuser
   * instead of going black. Returns led_count * render_markers_per_led_ opaque colors.
   */
  std::vector<gz::math::Color> ResampleLedColors(
    const std::vector<gz::math::Color> & led_colors) const;

  /**
   * @brief Alpha-composite a color over strip_base_color_, returning an opaque color.
   */
  gz::math::Color CompositeOverBase(const gz::math::Color & color) const;

  /**
   * @brief Interpolate a point at the given arc-length distance along the points polyline.
   */
  gz::math::Vector3d PolylinePointAt(double distance) const;

  /**
   * @brief Compute the centerline vertices of an arc-length segment of the points polyline: its
   * start and end points plus any polyline vertices falling between them, so the ribbon follows the
   * polyline exactly (no gaps at corners).
   *
   * @param start_distance Arc-length distance of the segment start from the polyline start
   * @param end_distance Arc-length distance of the segment end from the polyline start
   */
  std::vector<gz::math::Vector3d> PolylineSegmentVertices(
    double start_distance, double end_distance) const;

  /**
   * @brief Create a single LED of the strip as a double-sided triangle ribbon following the given
   * centerline vertices (expressed in the light frame), expanded vertically by the strip height.
   *
   * @param marker_msg The marker message to populate (one entry of the batched marker array)
   * @param id The unique ID of the marker (if not unique, the marker will replace the existing one)
   * @param strip_pose The pose of the strip (light) the ribbon vertices are relative to
   * @param color The color of the LED
   * @param centerline The centerline vertices of the LED
   */
  void CreateRibbonMarker(
    gz::msgs::Marker & marker_msg, const uint id, const gz::math::Pose3d & strip_pose,
    const gz::math::Color & color, const std::vector<gz::math::Vector3d> & centerline);

  /**
   * @brief Create a marker element (single LED from LED Strip)
   *
   * @param marker_msg The marker message to populate (one entry of the batched marker array)
   * @param id The unique ID of the marker (if not unique, the marker will replace the existing one)
   * @param pose The pose of the marker
   * @param color The color of the marker
   * @param size The size of the marker
   */
  void CreateMarker(
    gz::msgs::Marker & marker_msg, const uint id, const gz::math::Pose3d pose,
    const gz::math::Color & color, const gz::math::Vector3d size);

  std::string light_name_;
  std::string image_topic_;
  std::string ns_ = "";
  std::string model_name_ = "";
  double frequency_ = 10.0;
  double marker_width_ = 1.0;
  double marker_height_ = 1.0;

  // Optional subrange of the channel frame displayed by this instance ("first-last", a reversed
  // range flips the LED order). Defaults to the whole frame.
  int led_first_ = -1;
  int led_last_ = -1;

  // Number of rendered marker cells per LED used to interpolate the diffuser gradient. 1 disables
  // it (sharp per-LED markers).
  int render_markers_per_led_ = 1;

  // Diffuser base color the strip is composited over, so animation transparency reveals it instead
  // of the black scene. Defaults to white.
  gz::math::Color strip_base_color_ = gz::math::Color::White;

  // Optional polyline (in the light frame) the LEDs are distributed along. When empty, the strip
  // is a straight line along the Y axis of the light frame (legacy behavior).
  std::vector<gz::math::Vector3d> points_;
  std::vector<double> polyline_seg_lengths_;
  double polyline_length_ = 0.0;

  // A maximal span of consecutive render cells [start, end] sharing one color, rendered as a single
  // marker.
  struct ColorRun
  {
    size_t start;
    size_t end;
    gz::math::Color color;
    bool operator==(const ColorRun & other) const
    {
      return start == other.start && end == other.end && color == other.color;
    }
  };

  bool new_image_available_ = false;
  // Color runs sent in the previous frame. A run is re-published only when it changed, and surplus
  // markers are deleted, so a static animation generates no /marker traffic while the markers stay
  // attached to the (possibly moving) robot via their parent.
  std::vector<ColorRun> last_runs_;
  std::string last_rendered_data_;
  gz::msgs::Light light_cmd_;
  realtime_tools::RealtimeThreadSafeBox<gz::msgs::Image> last_image_;
  gz::sim::Entity light_entity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::chrono::steady_clock::duration last_update_time_{std::chrono::seconds(
    1)};  // Avoid initialization errors when the robot is not yet spawned on the scene.
};

}  // namespace husarion_ugv_gazebo

#endif  // HUSARION_UGV_GAZEBO_HUSARION_UGV_GAZEBO_LED_STRIP_HPP_
