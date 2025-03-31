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

#include <chrono>
#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_lights/lights_controller_node.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

class ControllerNodeWrapper : public husarion_ugv_lights::LightsControllerNode
{
public:
  ControllerNodeWrapper(const rclcpp::NodeOptions & options) : LightsControllerNode(options) {}

  void InitializeLEDPanels(const YAML::Node & panels_description)
  {
    return LightsControllerNode::InitializeLEDPanels(panels_description);
  }

  void InitializeLEDSegments(const YAML::Node & segments_description, const float controller_freq)
  {
    return LightsControllerNode::InitializeLEDSegments(segments_description, controller_freq);
  }

  void LoadAnimation(const YAML::Node & animation_description)
  {
    return LightsControllerNode::LoadAnimation(animation_description);
  }

  void AddAnimationToQueue(
    const std::size_t animation_id, const bool repeating, const std::string & param = "")
  {
    return LightsControllerNode::AddAnimationToLayer(animation_id, repeating, param);
  }
};

class TestLightsControllerNode : public testing::Test
{
public:
  TestLightsControllerNode();
  ~TestLightsControllerNode();

protected:
  void CreateLEDConfig(const std::filesystem::path file_path);

  static constexpr std::size_t kTestNumberOfLeds = 10;
  static constexpr std::size_t kTestChannel = 1;
  static constexpr char kTestSegmentName[] = "test";
  static constexpr char kTestSegmentLedRange[] = "0-9";

  std::filesystem::path animations_config_path_;
  std::shared_ptr<ControllerNodeWrapper> lights_controller_node_;
};

TestLightsControllerNode::TestLightsControllerNode()
{
  animations_config_path_ = testing::TempDir() + "/animations.yaml";

  CreateLEDConfig(animations_config_path_);

  std::vector<rclcpp::Parameter> params;
  params.push_back(rclcpp::Parameter("animations_config_path", animations_config_path_));

  rclcpp::NodeOptions options;
  options.parameter_overrides(params);

  lights_controller_node_ = std::make_shared<ControllerNodeWrapper>(options);
}

TestLightsControllerNode::~TestLightsControllerNode()
{
  std::filesystem::remove(animations_config_path_);
}

void TestLightsControllerNode::CreateLEDConfig(const std::filesystem::path file_path)
{
  YAML::Node panel;
  panel["channel"] = kTestChannel;
  panel["number_of_leds"] = kTestNumberOfLeds;

  YAML::Node segment;
  segment["name"] = kTestSegmentName;
  segment["channel"] = kTestChannel;
  segment["led_range"] = kTestSegmentLedRange;

  YAML::Node segments_map;
  segments_map["test"] = std::vector<std::string>(1, kTestSegmentName);

  YAML::Node animation;
  animation["image"] = "$(find husarion_ugv_lights)/test/files/strip01_red.png";
  animation["duration"] = 2;

  YAML::Node animation_desc;
  animation_desc["type"] = "husarion_ugv_lights::ImageAnimation";
  animation_desc["segments"] = "test";
  animation_desc["animation"] = animation;

  YAML::Node led_animation_0;
  led_animation_0["id"] = 0;
  led_animation_0["animations"] = std::vector<YAML::Node>(1, animation_desc);
  YAML::Node led_animation_1;
  led_animation_1["id"] = 1;
  led_animation_1["animations"] = std::vector<YAML::Node>(1, animation_desc);
  led_animation_1["priority"] = 2;

  std::vector<YAML::Node> led_animations;
  led_animations.push_back(led_animation_0);
  led_animations.push_back(led_animation_1);

  YAML::Node animations_config;
  animations_config["panels"] = std::vector<YAML::Node>(1, panel);
  animations_config["segments"] = std::vector<YAML::Node>(1, segment);
  animations_config["segments_map"] = segments_map;
  animations_config["led_animations"] = led_animations;

  YAML::Emitter out;
  out << animations_config;

  std::ofstream fout(file_path);
  if (fout.is_open()) {
    fout << out.c_str();
    fout.close();
  } else {
    throw std::runtime_error("Failed to create file: " + std::string(file_path));
  }
}

TEST_F(TestLightsControllerNode, InitializeLEDPanelsThrowRepeatingChannel)
{
  YAML::Node panel_1_desc;
  panel_1_desc["channel"] = kTestChannel;
  panel_1_desc["number_of_leds"] = kTestNumberOfLeds;

  YAML::Node panel_2_desc;
  panel_2_desc["channel"] = kTestChannel;
  panel_2_desc["number_of_leds"] = kTestNumberOfLeds;

  std::vector<YAML::Node> panels;
  panels.push_back(panel_1_desc);
  panels.push_back(panel_2_desc);

  YAML::Node panels_desc;
  panels_desc["panels"] = panels;

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->InitializeLEDPanels(panels_desc["panels"]); },
    "Multiple panels with channel nr"));
}

TEST_F(TestLightsControllerNode, InitializeLEDSegmentsThrowRepeatingName)
{
  YAML::Node segment_1_desc;
  segment_1_desc["name"] = kTestSegmentName;
  segment_1_desc["channel"] = kTestChannel;
  segment_1_desc["led_range"] = kTestSegmentLedRange;

  YAML::Node segment_2_desc;
  segment_2_desc["name"] = kTestSegmentName;
  segment_2_desc["channel"] = kTestChannel;
  segment_2_desc["led_range"] = kTestSegmentLedRange;

  std::vector<YAML::Node> segments;
  segments.push_back(segment_1_desc);
  segments.push_back(segment_2_desc);

  YAML::Node segments_desc;
  segments_desc["segments"] = segments;

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->InitializeLEDSegments(segments_desc["segments"], 50.0); },
    "Multiple segments with given name found"));
}

TEST_F(TestLightsControllerNode, LoadAnimationInvalidPriority)
{
  YAML::Node led_animation_desc;
  led_animation_desc["id"] = 11;
  led_animation_desc["priority"] = 40;

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->LoadAnimation(led_animation_desc); },
    "Invalid LED animation priority"));

  led_animation_desc["priority"] = 4;

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->LoadAnimation(led_animation_desc); },
    "Invalid LED animation priority"));
}

TEST_F(TestLightsControllerNode, LoadAnimationThrowRepeatingID)
{
  YAML::Node led_animation_desc;
  led_animation_desc["id"] = 11;
  led_animation_desc["animations"] = std::vector<YAML::Node>();

  ASSERT_NO_THROW(lights_controller_node_->LoadAnimation(led_animation_desc));

  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->LoadAnimation(led_animation_desc); },
    "Animation with given ID already exists"));
}

TEST_F(TestLightsControllerNode, AddAnimationToQueueThrowBadAnimationID)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { lights_controller_node_->AddAnimationToQueue(99, false); }, "No animation with ID:"));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
