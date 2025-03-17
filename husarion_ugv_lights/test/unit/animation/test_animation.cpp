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

#include "husarion_ugv_lights/animation/animation.hpp"

class AnimationWrapper : public husarion_ugv_lights::Animation
{
public:
  AnimationWrapper() {}

  void Initialize(
    const YAML::Node & animation_description, const std::size_t num_led,
    const float controller_frequency) override
  {
    Animation::Initialize(animation_description, num_led, controller_frequency);
    frame_size_ = this->GetNumberOfLeds() * 4;
  }

  std::vector<std::uint8_t> UpdateFrame() override
  {
    return std::vector<std::uint8_t>(frame_size_, 147);
  }

  std::vector<std::uint8_t> InvertRGBAFrame(const std::vector<std::uint8_t> & frame) const
  {
    return Animation::InvertRGBAFrame(frame);
  }

  std::size_t GetAnimationLength() const { return Animation::GetAnimationLength(); }
  std::size_t GetAnimationIteration() const { return Animation::GetAnimationIteration(); }
  void SetFrameSize(const std::size_t frame_size) { frame_size_ = frame_size; }

private:
  std::size_t frame_size_;
};

class TestAnimation : public testing::Test
{
public:
  TestAnimation() { animation_ = std::make_unique<AnimationWrapper>(); }
  ~TestAnimation() {}

protected:
  std::unique_ptr<AnimationWrapper> animation_;
};

TEST_F(TestAnimation, Initialize)
{
  YAML::Node animation_description;

  // missing duration
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  // invalid duration
  animation_description["duration"] = "-1.0";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::out_of_range);
  animation_description["duration"] = "word";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  // invalid animation length
  animation_description["duration"] = "0.1";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 1.0), std::runtime_error);

  animation_description["duration"] = "2.0";
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));

  // invalid repeat
  animation_description["repeat"] = "-2";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);
  animation_description["repeat"] = "1.1";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  // exceeded anim display duration
  animation_description["repeat"] = "6";
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  animation_description["repeat"] = "2";
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));
}

TEST_F(TestAnimation, CheckInitialValues)
{
  const std::size_t num_led = 10;
  const float controller_frequency = 10.0;
  YAML::Node animation_description = YAML::Load("{duration: 2.0, repeat: 2}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, controller_frequency));

  EXPECT_EQ(num_led, animation_->GetNumberOfLeds());
  EXPECT_EQ(std::size_t(20), animation_->GetAnimationLength());
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  EXPECT_FLOAT_EQ(0.0, animation_->GetProgress());
}

TEST_F(TestAnimation, Reset)
{
  ASSERT_NO_THROW(animation_->Initialize(YAML::Load("{duration: 2.0}"), 10, 10.0f));

  // update animation
  ASSERT_NO_THROW(animation_->Update());
  EXPECT_NE(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_NE(0.0F, animation_->GetProgress());

  // reset animation
  animation_->Reset();
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FLOAT_EQ(0.0, animation_->GetProgress());
  EXPECT_FALSE(animation_->IsFinished());
}

TEST_F(TestAnimation, UpdateWithInvalidFrameSize)
{
  ASSERT_NO_THROW(animation_->Initialize(YAML::Load("{duration: 2.0}"), 10, 10.0f));
  EXPECT_NO_THROW(animation_->Update());

  animation_->SetFrameSize(11);
  EXPECT_THROW(animation_->Update(), std::runtime_error);
}

TEST_F(TestAnimation, Update)
{
  const std::size_t num_led = 10;
  const float controller_frequency = 10.0;
  YAML::Node animation_description = YAML::Load("{duration: 2.0, repeat: 2}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, controller_frequency));

  // check random progress
  for (std::size_t i = 0; i < 5; i++) {
    ASSERT_NO_THROW(animation_->Update());
  }
  EXPECT_EQ(std::size_t(5), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  float expected_progress = 5.0 / (20 * 2);
  EXPECT_FLOAT_EQ(expected_progress, animation_->GetProgress());

  animation_->Reset();

  // reach end of first loop of animation
  for (std::size_t i = 0; i < 20; i++) {
    ASSERT_NO_THROW(animation_->Update());
  }
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_FALSE(animation_->IsFinished());
  expected_progress = 20.0 / (20 * 2);
  EXPECT_FLOAT_EQ(expected_progress, animation_->GetProgress());

  // reach animation end
  for (std::size_t i = 0; i < 20; i++) {
    ASSERT_NO_THROW(animation_->Update());
  }
  EXPECT_EQ(std::size_t(0), animation_->GetAnimationIteration());
  EXPECT_TRUE(animation_->IsFinished());
  EXPECT_FLOAT_EQ(1.0, animation_->GetProgress());

  // after reaching animation end Update() method when invoked should return frame filled with 0
  animation_->Update();
  auto frame = animation_->GetFrame();
  EXPECT_EQ(num_led * 4, frame.size());
  for (std::size_t i = 0; i < num_led * 3; i++) {
    EXPECT_EQ(0, frame[i]);
  }
}

TEST_F(TestAnimation, InvertRGBAFrame)
{
  std::vector<std::uint8_t> test_frame = {0,   10,  20,  255, 30,  40,  50,  255,
                                          60,  70,  80,  255, 100, 110, 120, 255,
                                          130, 140, 150, 255, 160, 170, 180, 255};
  std::vector<std::uint8_t> expected_frame = {160, 170, 180, 255, 130, 140, 150, 255,
                                              100, 110, 120, 255, 60,  70,  80,  255,
                                              30,  40,  50,  255, 0,   10,  20,  255};

  auto inverted_frame = animation_->InvertRGBAFrame(test_frame);
  for (std::size_t i = 0; i < inverted_frame.size(); i++) {
    EXPECT_EQ(expected_frame[i], inverted_frame[i]);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
