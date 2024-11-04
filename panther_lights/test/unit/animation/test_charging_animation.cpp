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

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "panther_lights/animation/charging_animation.hpp"

class ChargingAnimationWrapper : public panther_lights::ChargingAnimation
{
public:
  ChargingAnimationWrapper() {}

  std::vector<uint8_t> UpdateFrame() { return ChargingAnimation::UpdateFrame(); }

  std::array<std::uint8_t, 3> HSVtoRGB(const float h, const float s, const float v) const
  {
    return ChargingAnimation::HSVtoRGB(h, s, v);
  }

  std::vector<std::uint8_t> CreateRGBAFrame(
    const std::array<std::uint8_t, 3> color, const float brightness) const
  {
    return ChargingAnimation::CreateRGBAFrame(color, brightness);
  }

  std::size_t GetAnimationLength() const { return ChargingAnimation::GetAnimationLength(); }
  std::size_t GetAnimationIteration() const { return ChargingAnimation::GetAnimationIteration(); }
};

class TestChargingAnimation : public testing::Test
{
public:
  TestChargingAnimation();
  ~TestChargingAnimation();

protected:
  void TestRGBColor(
    const std::array<std::uint8_t, 3UL> & color, std::uint8_t r, std::uint8_t g,
    std::uint8_t b) const;
  void TestRGBAFrame(
    const std::vector<std::uint8_t> & frame, std::uint8_t r, std::uint8_t g, std::uint8_t b,
    std::uint8_t a);

  std::unique_ptr<ChargingAnimationWrapper> animation_;
};

TestChargingAnimation::TestChargingAnimation()
{
  animation_ = std::make_unique<ChargingAnimationWrapper>();
}

TestChargingAnimation::~TestChargingAnimation() {}

void TestChargingAnimation::TestRGBColor(
  const std::array<std::uint8_t, 3UL> & color, std::uint8_t r, std::uint8_t g, std::uint8_t b) const
{
  EXPECT_EQ(r, color[0]);
  EXPECT_EQ(g, color[1]);
  EXPECT_EQ(b, color[2]);
}

void TestChargingAnimation::TestRGBAFrame(
  const std::vector<std::uint8_t> & frame, std::uint8_t r, std::uint8_t g, std::uint8_t b,
  std::uint8_t a)
{
  for (std::size_t i = 0; i < frame.size(); i += 4) {
    EXPECT_EQ(r, frame[i]);
    EXPECT_EQ(g, frame[i + 1]);
    EXPECT_EQ(b, frame[i + 2]);
    EXPECT_EQ(a, frame[i + 3]);
  }
}

TEST_F(TestChargingAnimation, HSVtoRGB)
{
  // saturation equal 0 with max value should return white
  auto color = animation_->HSVtoRGB(0.0, 0.0, 1.0);
  TestRGBColor(color, 255, 255, 255);

  // red
  color = animation_->HSVtoRGB(0.0, 1.0, 1.0);
  TestRGBColor(color, 255, 0, 0);

  // green
  color = animation_->HSVtoRGB(120.0 / 360.0, 1.0, 1.0);
  TestRGBColor(color, 0, 255, 0);

  // blue
  color = animation_->HSVtoRGB(240.0 / 360.0, 1.0, 1.0);
  TestRGBColor(color, 0, 0, 255);

  // test colors for each of 6 hue segments
  color = animation_->HSVtoRGB(51.0f / 360.0f, 0.8f, 0.3f);
  TestRGBColor(color, 76, 67, 15);

  color = animation_->HSVtoRGB(77.0f / 360.0f, 0.5f, 0.7f);
  TestRGBColor(color, 153, 178, 89);

  color = animation_->HSVtoRGB(150.0f / 360.0f, 0.2f, 0.8f);
  TestRGBColor(color, 163, 204, 183);

  color = animation_->HSVtoRGB(222.0f / 360.0f, 0.8f, 0.5f);
  TestRGBColor(color, 25, 56, 127);

  color = animation_->HSVtoRGB(291.0f / 360.0f, 0.7f, 0.4f);
  TestRGBColor(color, 91, 30, 102);

  color = animation_->HSVtoRGB(345.0f / 360.0f, 0.1f, 0.9f);
  TestRGBColor(color, 229, 206, 212);
}

TEST_F(TestChargingAnimation, CreateRGBAFrame)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description = YAML::Load("{duration: 2.0}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));

  std::array<std::uint8_t, 3> color{70, 120, 91};
  auto frame = animation_->CreateRGBAFrame(color, 1.0);
  ASSERT_EQ(num_led * 4, frame.size());
  TestRGBAFrame(frame, 70, 120, 91, 255);

  // reduce brightness
  frame = animation_->CreateRGBAFrame(color, 0.5);
  ASSERT_EQ(num_led * 4, frame.size());
  TestRGBAFrame(frame, 35, 60, 45, 255);
}

TEST_F(TestChargingAnimation, SetParam)
{
  EXPECT_THROW(animation_->SetParam("not_a_number"), std::runtime_error);
  EXPECT_NO_THROW(animation_->SetParam("0.7"));
}

TEST_F(TestChargingAnimation, UpdateFrame)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description = YAML::Load("{duration: 2.0}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));

  // UpdateFrame depends on parent class variables which can be only updated using Update method.
  // For full battery whole animation should be a solid green color
  animation_->SetParam("1.0");
  for (std::uint8_t i = 0; i < animation_->GetAnimationLength(); i++) {
    animation_->Update();
    auto frame = animation_->GetFrame();
    ASSERT_EQ(num_led * 4, frame.size());
    TestRGBAFrame(frame, 0, 255, 0, 255);
  }

  animation_->Reset();
  animation_->SetParam("0.5");
  std::vector<std::uint8_t> frame;

  // the beginning of the animation should be dark
  animation_->Update();
  frame = animation_->GetFrame();
  ASSERT_EQ(num_led * 4, frame.size());
  TestRGBAFrame(frame, 0, 0, 0, 255);

  // reach middle of the animation
  while (animation_->GetAnimationIteration() < animation_->GetAnimationLength() / 2) {
    animation_->Update();
    frame = animation_->GetFrame();
  }

  // the middle of animation for param 0.5 should be yellow
  ASSERT_EQ(num_led * 4, frame.size());
  TestRGBAFrame(frame, 255, 255, 0, 255);

  // reach end of the animation
  while (!animation_->IsFinished()) {
    animation_->Update();
    frame = animation_->GetFrame();
  }

  // the end of the animation should be dark
  ASSERT_EQ(num_led * 4, frame.size());
  TestRGBAFrame(frame, 0, 0, 0, 255);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
