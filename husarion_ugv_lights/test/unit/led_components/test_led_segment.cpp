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
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/led_components/led_segment.hpp"
#include "husarion_ugv_utils/test/test_utils.hpp"

class LEDSegmentWrapper : public husarion_ugv_lights::LEDSegment
{
public:
  LEDSegmentWrapper(const YAML::Node & segment_description, const float controller_frequency)
  : LEDSegment(segment_description, controller_frequency)
  {
  }

  std::shared_ptr<husarion_ugv_lights::Animation> GetAnimation() const { return animation_; }
  std::shared_ptr<husarion_ugv_lights::Animation> GetDefaultAnimation() const
  {
    return default_animation_;
  }
};

class TestLEDSegment : public testing::Test
{
public:
  TestLEDSegment();
  ~TestLEDSegment() {}

protected:
  std::shared_ptr<LEDSegmentWrapper> led_segment_;

  const float controller_freq = 50.0;
  const std::size_t segment_led_num_ = 10;
};

TestLEDSegment::TestLEDSegment()
{
  const auto segment_desc =
    YAML::Load("{led_range: 0-" + std::to_string(segment_led_num_ - 1) + ", channel: 1}");
  led_segment_ = std::make_shared<LEDSegmentWrapper>(segment_desc, controller_freq);
}

YAML::Node CreateSegmentDescription(const std::string & led_range, const std::string & channel)
{
  return YAML::Load("{led_range: " + led_range + ", channel: " + channel + "}");
}

TEST(TestLEDSegmentInitialization, DescriptionMissingRequiredKey)
{
  auto segment_desc = YAML::Load("");
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Missing 'channel' in description"));

  segment_desc = YAML::Load("channel: 0");
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Missing 'led_range' in description"));
}

TEST(TestLEDSegmentInitialization, InvalidChannelExpression)
{
  auto segment_desc = CreateSegmentDescription("0-10", "s1");
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Failed to convert 'channel' key"));

  segment_desc["channel"] = "-1";
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Failed to convert 'channel' key"));
}

TEST(TestLEDSegmentInitialization, InvalidLedRangeExpression)
{
  auto segment_desc = CreateSegmentDescription("010", "1");
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "No '-' character found in the led_range expression"));

  segment_desc["led_range"] = "s0-10";
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Error converting string to integer"));

  segment_desc["led_range"] = "0-p10";
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::invalid_argument>(
    [segment_desc]() { husarion_ugv_lights::LEDSegment(segment_desc, 10.0); },
    "Error converting string to integer"));
}

TEST(TestLEDSegmentInitialization, ValidDescription)
{
  const auto segment_desc = CreateSegmentDescription("0-10", "1");
  EXPECT_NO_THROW(husarion_ugv_lights::LEDSegment(segment_desc, 10.0));
}

TEST(TestLEDSegmentInitialization, FirstLedPosition)
{
  auto segment_desc = CreateSegmentDescription("0-10", "1");
  std::shared_ptr<husarion_ugv_lights::LEDSegment> led_segment;

  ASSERT_NO_THROW(
    led_segment = std::make_shared<husarion_ugv_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(0), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "5-11";
  led_segment.reset();
  ASSERT_NO_THROW(
    led_segment = std::make_shared<husarion_ugv_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(5 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "10-10";
  led_segment.reset();
  ASSERT_NO_THROW(
    led_segment = std::make_shared<husarion_ugv_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(10 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "13-5";
  led_segment.reset();
  ASSERT_NO_THROW(
    led_segment = std::make_shared<husarion_ugv_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(5 * 4), led_segment->GetFirstLEDPosition());

  segment_desc["led_range"] = "17-0";
  led_segment.reset();
  ASSERT_NO_THROW(
    led_segment = std::make_shared<husarion_ugv_lights::LEDSegment>(segment_desc, 10.0));
  EXPECT_EQ(std::size_t(0), led_segment->GetFirstLEDPosition());
}

TEST_F(TestLEDSegment, GetAnimationFrameNoAnimation)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { led_segment_->GetAnimationFrame(); }, "Segment animation not defined"));
}

TEST_F(TestLEDSegment, GetAnimationProgressNoAnimation)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { led_segment_->GetAnimationProgress(); }, "Segment animation not defined"));
}

TEST_F(TestLEDSegment, ResetAnimationNoAnimation)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { led_segment_->ResetAnimation(); }, "Segment animation not defined"));
}

TEST_F(TestLEDSegment, GetAnimationBrightnessNoAnimation)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { led_segment_->GetAnimationBrightness(); }, "Segment animation not defined"));
}

TEST_F(TestLEDSegment, SetAnimationInvalidType)
{
  const YAML::Node animation_desc;
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() {
      led_segment_->SetAnimation("husarion_ugv_lights::WrongAnimationType}", animation_desc, false);
    },
    "The plugin failed to load. Error: "));
}

TEST_F(TestLEDSegment, SetAnimationFailAnimationInitialization)
{
  const auto animation_desc = YAML::Load("{type: husarion_ugv_lights::ImageAnimation}");
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() {
      led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", animation_desc, false);
    },
    "Failed to initialize animation: "));
}

TEST_F(TestLEDSegment, SetAnimation)
{
  // test each known animtion type
  const auto image_anim_desc = YAML::Load(
    "{image: $(find husarion_ugv_lights)/test/files/animation.png, "
    "duration: 2}");
  const auto charging_anim_desc = YAML::Load("{duration: 2}");

  EXPECT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", image_anim_desc, false));

  EXPECT_NO_THROW(led_segment_->SetAnimation(
    "husarion_ugv_lights::ChargingAnimation", charging_anim_desc, false, "0.5"));
}

TEST_F(TestLEDSegment, SetAnimationRepeating)
{
  const auto anim_desc = YAML::Load(
    "{image: $(find husarion_ugv_lights)/test/files/animation.png, "
    "duration: 2}");
  ASSERT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, false));

  EXPECT_TRUE(led_segment_->GetDefaultAnimation().get() == nullptr);

  ASSERT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, true));

  EXPECT_TRUE(led_segment_->GetDefaultAnimation().get() != nullptr);
  EXPECT_TRUE(led_segment_->IsAnimationFinished());
}

TEST_F(TestLEDSegment, UpdateAnimationAnimationNotSet)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { led_segment_->UpdateAnimation(); }, "Segment animation not defined"));
}

TEST_F(TestLEDSegment, UpdateAnimation)
{
  const auto anim_desc = YAML::Load(
    "{image: $(find husarion_ugv_lights)/test/files/animation.png, "
    "duration: 2}");
  ASSERT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, false));
  EXPECT_NO_THROW(led_segment_->UpdateAnimation());
  EXPECT_EQ(segment_led_num_ * 4, led_segment_->GetAnimationFrame().size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST_F(TestLEDSegment, ResetDefaultAnimationWhenNewArrive)
{
  const auto anim_desc = YAML::Load(
    "{image: $(find husarion_ugv_lights)/test/files/animation.png, "
    "duration: 2}");
  ASSERT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, true));

  auto default_anim = led_segment_->GetDefaultAnimation();
  while (!default_anim->IsFinished()) {
    ASSERT_NO_THROW(led_segment_->UpdateAnimation());
  }

  // add new animation, and check if default animation was reset
  ASSERT_NO_THROW(
    led_segment_->SetAnimation("husarion_ugv_lights::ImageAnimation", anim_desc, false));

  EXPECT_FALSE(default_anim->IsFinished());
}
