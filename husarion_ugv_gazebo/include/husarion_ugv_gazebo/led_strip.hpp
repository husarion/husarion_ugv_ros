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

#ifndef HUSARION_UGV_GAZEBO_LED_STRIP_HPP_
#define HUSARION_UGV_GAZEBO_LED_STRIP_HPP_

#include <chrono>
#include <string>

#include <realtime_tools/realtime_box.hpp>

#include <gz/math/Color.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/EventManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Light.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

#include <gz/msgs/image.pb.h>
#include <gz/msgs/light.pb.h>

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
   * @brief Create a marker element (single LED from LED Strip)
   *
   * @param id The unique ID of the marker (if not unique, the marker will replace the existing one)
   * @param pose The pose of the marker
   * @param color The color of the marker
   * @param size The size of the marker
   */
  void CreateMarker(
    const uint id, const gz::math::Pose3d pose, const gz::math::Color & color,
    const gz::math::Vector3d size);

  std::string light_name_;
  std::string image_topic_;
  std::string ns_ = "";
  double frequency_ = 10.0;
  double marker_width_ = 1.0;
  double marker_height_ = 1.0;

  bool new_image_available_ = false;
  gz::msgs::Light light_cmd_;
  realtime_tools::RealtimeBox<gz::msgs::Image> last_image_;
  gz::sim::Entity light_entity_{gz::sim::kNullEntity};
  gz::transport::Node node_;
  std::chrono::steady_clock::duration last_update_time_{std::chrono::seconds(
    1)};  // Avoid initialization errors when the robot is not yet spawned on the scene.
};

}  // namespace husarion_ugv_gazebo

#endif  // HUSARION_UGV_GAZEBO_LED_STRIP_HPP_
