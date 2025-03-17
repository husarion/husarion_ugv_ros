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

#ifndef HUSARION_UGV_LIGHTS_LIGHTS_CONTROLLER_NODE_HPP_
#define HUSARION_UGV_LIGHTS_LIGHTS_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "husarion_ugv_msgs/srv/set_led_animation.hpp"

#include "husarion_ugv_lights/animation/animation.hpp"
#include "husarion_ugv_lights/led_components/led_animation.hpp"
#include "husarion_ugv_lights/led_components/segment_converter.hpp"
#include "husarion_ugv_lights/lights_controller_parameters.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_lights
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLEDAnimationSrv = husarion_ugv_msgs::srv::SetLEDAnimation;

class LightsControllerNode : public rclcpp::Node
{
public:
  LightsControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~LightsControllerNode() {}

protected:
  /**
   * @brief Initializes LED panel based on YAML description. This LED panel is a representation of
   * the real panel of the robot
   *
   * @param panels_description YAML description of the panel, must contain 'channel' and
   * 'number_of_leds' keys
   *
   * @exception std::runtime_error if initialization of the LED panel fails
   */
  void InitializeLEDPanels(const YAML::Node & panels_description);

  /**
   * @brief Initializes LED segment based on YAML description. This LED segment is a representation
   * of the abstract segment located on the panel of the robot
   *
   * @param segments_description YAML description of the segment, must contain 'name', 'channel' and
   * 'led_range' keys
   * @param controller_freq Frequency at which animations will be processed
   *
   * @exception std::runtime_error if initialization of the LED segment fails
   */
  void InitializeLEDSegments(const YAML::Node & segments_description, const float controller_freq);

  /**
   * @brief Initializes LED segments map based on YAML description. This assigns list with segments
   * names to abstract names that can be used with animations to specify on which segments animation
   * should be displayed
   *
   * @param segments_map_description YAML description of the segments map
   */
  void InitializeLEDSegmentsMap(const YAML::Node & segments_map_description);

  /**
   * @brief Adds animations to an unordered map with animations
   *
   * @param animations_description YAML description with list of animations
   *
   * @exception std::runtime_error if fails to load an animation or animation with given ID already
   * exists in the map
   */
  void LoadDefaultAnimations(const YAML::Node & animations_description);

  /**
   * @brief Adds animations to an unordered map with animations
   *
   * @param user_led_animations_path path to YAML file with user animations description
   */
  void LoadUserAnimations(const std::string & user_led_animations_path);

  /**
   * @brief Adds animation to an unordered map with animations
   *
   * @param animations_description YAML description of the animation
   *
   * @exception std::runtime_error if fails to load an animation or animation with given ID already
   * exists in the map
   */
  void LoadAnimation(const YAML::Node & animation_description);

  /**
   * @brief Updates all segments animations, converts then into panel frames and publishes panel
   * frames on respective topics
   */
  void UpdateAndPublishAnimation();

  /**
   * @brief Add animation to LED animations queue
   *
   * @param animation_id ID of the animations
   * @param repeating Whether animations should repeat
   * @param param Optional animation parameter
   *
   * @exception std::runtime_error if no animation with given ID exists
   */
  void AddAnimationToLayer(
    const std::size_t animation_id, const bool repeating, const std::string & param);

  /**
   * @brief Add animations to LED segments based on LED animation
   *
   * @param led_animation LED animation
   *
   * @exception std::runtime_error animation has invalid segment name or it fails to load
   */
  void SetLEDAnimation(const std::shared_ptr<LEDAnimation> & led_animation);

private:
  void PublishPanelFrame(const std::size_t channel);
  void SetLEDAnimationCB(
    const SetLEDAnimationSrv::Request::SharedPtr & request,
    SetLEDAnimationSrv::Response::SharedPtr response);
  void ControllerTimerCB();

  std::unordered_map<std::size_t, std::shared_ptr<LEDPanel>> led_panels_;
  std::unordered_map<std::size_t, rclcpp::Publisher<ImageMsg>::SharedPtr> panel_publishers_;
  std::unordered_map<std::string, std::shared_ptr<LEDSegment>> segments_;
  std::unordered_map<std::string, std::vector<std::string>> segments_map_;
  std::unordered_map<std::size_t, LEDAnimationDescription> animations_descriptions_;
  std::shared_ptr<SegmentConverter> segment_converter_;

  std::shared_ptr<lights_controller::ParamListener> param_listener_;
  lights_controller::Params params_;

  rclcpp::Service<SetLEDAnimationSrv>::SharedPtr set_led_animation_server_;
  rclcpp::TimerBase::SharedPtr controller_timer_;

  bool animation_finished_ = true;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LIGHTS_CONTROLLER_NODE_HPP_
