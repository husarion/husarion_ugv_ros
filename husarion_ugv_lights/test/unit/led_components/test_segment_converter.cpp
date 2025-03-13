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

#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/led_components/led_panel.hpp"
#include "husarion_ugv_lights/led_components/led_segment.hpp"
#include "husarion_ugv_lights/led_components/segment_converter.hpp"

class TestSegmentConverter : public testing::Test
{
public:
  TestSegmentConverter()
  {
    segment_converter_ = std::make_unique<husarion_ugv_lights::SegmentConverter>();

    // create 2 basic panels with different number of leds
    led_panels_.insert({1, std::make_unique<husarion_ugv_lights::LEDPanel>(panel_1_num_led_)});
    led_panels_.insert({2, std::make_unique<husarion_ugv_lights::LEDPanel>(panel_2_num_led_)});
  }
  ~TestSegmentConverter() {}

protected:
  YAML::Node CreateSegmentDescription(
    const std::size_t first_led, const std::size_t last_led, const std::size_t channel) const;
  YAML::Node CreateImageAnimationDescription();

  std::size_t panel_1_num_led_ = 20;
  std::size_t panel_2_num_led_ = 30;

  std::unique_ptr<husarion_ugv_lights::SegmentConverter> segment_converter_;
  std::unordered_map<std::string, std::shared_ptr<husarion_ugv_lights::LEDSegment>> segments_;
  std::unordered_map<std::size_t, std::shared_ptr<husarion_ugv_lights::LEDPanel>> led_panels_;
};

YAML::Node TestSegmentConverter::CreateSegmentDescription(
  const std::size_t first_led, const std::size_t last_led, const std::size_t channel) const
{
  YAML::Node desc;
  desc["led_range"] = std::to_string(first_led) + "-" + std::to_string(last_led);
  desc["channel"] = channel;
  return desc;
}

YAML::Node TestSegmentConverter::CreateImageAnimationDescription()
{
  return YAML::Load("{image: $(find husarion_ugv_lights)/test/files/animation.png, duration: 2}");
}

TEST_F(TestSegmentConverter, ConvertInvalidChannel)
{
  segments_.emplace(
    "name",
    std::make_shared<husarion_ugv_lights::LEDSegment>(CreateSegmentDescription(0, 10, 123), 50.0));
  const auto anim_desc = CreateImageAnimationDescription();
  ASSERT_NO_THROW(
    segments_.at("name")->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, 0, false));

  EXPECT_THROW(segment_converter_->Convert(segments_, led_panels_), std::out_of_range);
}

TEST_F(TestSegmentConverter, ConvertInvalidLedRange)
{
  segments_.emplace(
    "name", std::make_shared<husarion_ugv_lights::LEDSegment>(
              CreateSegmentDescription(panel_1_num_led_, panel_1_num_led_ + 1, 1), 50.0));
  const auto anim_desc = CreateImageAnimationDescription();
  ASSERT_NO_THROW(
    segments_.at("name")->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, 0, false));

  EXPECT_THROW(segment_converter_->Convert(segments_, led_panels_), std::runtime_error);
}

TEST_F(TestSegmentConverter, ConvertSingleSegmentForEachPanel)
{
  segments_.emplace(
    "name_1", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, panel_1_num_led_ - 1, 1), 50.0));
  segments_.emplace(
    "name_2", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, panel_2_num_led_ - 1, 2), 50.0));

  const auto anim_desc = CreateImageAnimationDescription();

  for (auto & segment : segments_) {
    ASSERT_NO_THROW(
      segment.second->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, 0, false));
    ASSERT_NO_THROW(segment.second->UpdateAnimation());
  }

  EXPECT_NO_THROW(segment_converter_->Convert(segments_, led_panels_));
}

TEST_F(TestSegmentConverter, ConvertMultipleSegments)
{
  segments_.emplace(
    "name_1", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, std::size_t(panel_1_num_led_ / 2) - 1, 1), 50.0));
  segments_.emplace(
    "name_2",
    std::make_shared<husarion_ugv_lights::LEDSegment>(
      CreateSegmentDescription(std::size_t(panel_1_num_led_ / 2), panel_1_num_led_ - 1, 1), 50.0));

  segments_.emplace(
    "name_3", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, (panel_2_num_led_ / 4) - 1, 2), 50.0));
  segments_.emplace(
    "name_4",
    std::make_shared<husarion_ugv_lights::LEDSegment>(
      CreateSegmentDescription((panel_2_num_led_ / 4), (panel_2_num_led_ / 2) - 1, 2), 50.0));
  segments_.emplace(
    "name_5", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription((panel_2_num_led_ / 2), panel_2_num_led_ - 1, 2), 50.0));

  const auto anim_desc = CreateImageAnimationDescription();
  for (auto & segment : segments_) {
    ASSERT_NO_THROW(
      segment.second->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, 0, false));
    ASSERT_NO_THROW(segment.second->UpdateAnimation());
  }

  EXPECT_NO_THROW(segment_converter_->Convert(segments_, led_panels_));
}

TEST_F(TestSegmentConverter, ConvertNoThrowIfAnimationNotSet)
{
  segments_.emplace(
    "name_1", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, panel_1_num_led_ - 1, 1), 50.0));
  segments_.emplace(
    "name_2", std::make_shared<husarion_ugv_lights::LEDSegment>(
                CreateSegmentDescription(0, panel_2_num_led_ - 1, 2), 50.0));

  EXPECT_NO_THROW(segment_converter_->Convert(segments_, led_panels_));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
