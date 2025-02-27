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
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rclcpp/time.hpp"

#include "husarion_ugv_lights/led_components/led_animation.hpp"
#include "husarion_ugv_lights/led_components/led_segment.hpp"

class TestLEDAnimation : public testing::Test
{
public:
  TestLEDAnimation();
  ~TestLEDAnimation() {}

  void SetSegmentAnimations();

protected:
  static constexpr char kTestSegmentName1[] = "segment_1";
  static constexpr char kTestSegmentName2[] = "segment_2";

  std::shared_ptr<husarion_ugv_lights::LEDAnimation> led_anim_;
  std::unordered_map<std::string, std::shared_ptr<husarion_ugv_lights::LEDSegment>> segments_;
};

TestLEDAnimation::TestLEDAnimation()
{
  auto segment_1_desc = YAML::Load("{channel: 1, led_range: 0-10}");
  auto segment_2_desc = YAML::Load("{channel: 2, led_range: 0-10}");
  segments_.emplace(
    kTestSegmentName1, std::make_shared<husarion_ugv_lights::LEDSegment>(segment_1_desc, 50.0));
  segments_.emplace(
    kTestSegmentName2, std::make_shared<husarion_ugv_lights::LEDSegment>(segment_2_desc, 50.0));

  husarion_ugv_lights::AnimationDescription anim_desc;
  anim_desc.segments = {kTestSegmentName1, kTestSegmentName2};
  anim_desc.type = "husarion_ugv_lights::ImageAnimation";
  anim_desc.animation =
    YAML::Load("{image: $(find husarion_ugv_lights)/test/files/animation.png, duration: 2.0}");

  husarion_ugv_lights::LEDAnimationDescription led_anim_desc;
  led_anim_desc.id = 0;
  led_anim_desc.name = "TEST";
  led_anim_desc.priority = 1;
  led_anim_desc.timeout = 10.0;
  led_anim_desc.animations = {anim_desc};

  led_anim_ = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    led_anim_desc, segments_, rclcpp::Time(0));
}

void TestLEDAnimation::SetSegmentAnimations()
{
  const auto animations = led_anim_->GetAnimations();
  for (auto & animation : animations) {
    for (auto & segment : animation.segments) {
      segments_.at(segment)->SetAnimation(
        animation.type, animation.animation, 0, led_anim_->IsRepeating(), led_anim_->GetParam());
    }
  }
}

TEST(TestLEDAnimationInitialization, InvalidSegmentName)
{
  std::unordered_map<std::string, std::shared_ptr<husarion_ugv_lights::LEDSegment>> segments;

  husarion_ugv_lights::AnimationDescription anim_desc;
  anim_desc.segments = {"invalid_segment"};

  husarion_ugv_lights::LEDAnimationDescription led_anim_desc;
  led_anim_desc.animations = {anim_desc};

  EXPECT_THROW(
    std::make_shared<husarion_ugv_lights::LEDAnimation>(led_anim_desc, segments, rclcpp::Time(0)),
    std::runtime_error);
}

TEST(TestLEDAnimationInitialization, Successful)
{
  const char segment_name_1[] = "segment_1";
  const char segment_name_2[] = "segment_2";
  auto segment_1_desc = YAML::Load("{channel: 1, led_range: 0-10}");
  auto segment_2_desc = YAML::Load("{channel: 2, led_range: 0-10}");
  std::unordered_map<std::string, std::shared_ptr<husarion_ugv_lights::LEDSegment>> segments;

  segments.emplace(
    segment_name_1, std::make_shared<husarion_ugv_lights::LEDSegment>(segment_1_desc, 50.0));
  segments.emplace(
    segment_name_2, std::make_shared<husarion_ugv_lights::LEDSegment>(segment_2_desc, 50.0));

  husarion_ugv_lights::AnimationDescription anim_desc;
  anim_desc.segments = {segment_name_1, segment_name_2};
  anim_desc.type = "husarion_ugv_lights::ImageAnimation";
  anim_desc.animation =
    YAML::Load("{image: $(find husarion_ugv_lights)/test/files/animation.png, duration: 2.0}");

  husarion_ugv_lights::LEDAnimationDescription led_anim_desc;
  led_anim_desc.id = 0;
  led_anim_desc.name = "TEST";
  led_anim_desc.priority = 1;
  led_anim_desc.timeout = 10.0;
  led_anim_desc.animations = {anim_desc};

  EXPECT_NO_THROW(husarion_ugv_lights::LEDAnimation(led_anim_desc, segments, rclcpp::Time(0)));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
