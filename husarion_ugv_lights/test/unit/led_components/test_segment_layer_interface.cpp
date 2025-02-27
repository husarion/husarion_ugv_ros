// Copyright 2025 Husarion sp. z o.o.
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

#include <string>

#include "gtest/gtest.h"

#include "husarion_ugv_lights/led_components/segment_layer_interface.hpp"

class MockAnimation : public husarion_ugv_lights::Animation
{
public:
  std::vector<std::uint8_t> UpdateFrame() override { return this->frame_; }
  void SetFrame(const std::vector<std::uint8_t> & frame) { this->frame_ = frame; }
};

class SegmentLayerInterfaceWrapper : public husarion_ugv_lights::SegmentLayerInterface
{
public:
  SegmentLayerInterfaceWrapper(
    const std::size_t num_led, const bool invert_led_order, const float controller_frequency)
  : SegmentLayerInterface(num_led, invert_led_order, controller_frequency)
  {
  }

  void SetAnimation(
    const std::string & /*type*/, const YAML::Node & /*animation_description*/,
    const bool /*repeating*/, const std::string & /*param*/ = "") override
  {
    this->animation_ = std::make_shared<MockAnimation>();
  }

  void SetAnimation(std::shared_ptr<MockAnimation> & animation) { this->animation_ = animation; }

  void UpdateAnimation() override {}

  void SetAnimationFinished(const bool finished) { animation_finished_ = finished; }
};

class TestSegmentLayerInterface : public testing::Test
{
public:
  TestSegmentLayerInterface()
  {
    segment_layer_ = std::make_shared<SegmentLayerInterfaceWrapper>(kNumLed, false, 50.0);
  }
  ~TestSegmentLayerInterface() {}

protected:
  static constexpr unsigned kNumLed = 10;
  std::shared_ptr<SegmentLayerInterfaceWrapper> segment_layer_;
};

TEST_F(TestSegmentLayerInterface, GetAnimationFrameNoAnimation)
{
  std::vector<std::uint8_t> frame;

  EXPECT_NO_THROW(frame = this->segment_layer_->GetAnimationFrame());
  EXPECT_TRUE(frame.size() == this->kNumLed * 4);
  EXPECT_TRUE(
    std::all_of(frame.begin(), frame.end(), [](const std::uint8_t & led) { return led == 0; }));
}

TEST_F(TestSegmentLayerInterface, GetAnimationFrame)
{
  std::vector<std::uint8_t> frame;
  std::shared_ptr<MockAnimation> test_animation = std::make_shared<MockAnimation>();

  test_animation->SetFrame(std::vector<std::uint8_t>(this->kNumLed * 4, 255));
  this->segment_layer_->SetAnimation(test_animation);
  this->segment_layer_->SetAnimationFinished(false);

  EXPECT_NO_THROW(frame = this->segment_layer_->GetAnimationFrame());
  EXPECT_TRUE(frame.size() == this->kNumLed * 4);
  EXPECT_TRUE(
    std::all_of(frame.begin(), frame.end(), [](const std::uint8_t & led) { return led == 255; }));

  // After animation is finished, the frame should be empty
  this->segment_layer_->SetAnimationFinished(true);
  EXPECT_NO_THROW(frame = this->segment_layer_->GetAnimationFrame());
  EXPECT_TRUE(frame.size() == this->kNumLed * 4);
  EXPECT_TRUE(
    std::all_of(frame.begin(), frame.end(), [](const std::uint8_t & led) { return led == 0; }));
}
