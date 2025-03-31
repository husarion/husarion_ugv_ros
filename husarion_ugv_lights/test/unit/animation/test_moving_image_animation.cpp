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

#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/animation/moving_image_animation.hpp"

namespace gil = boost::gil;

class MovingImageAnimationWrapper : public husarion_ugv_lights::MovingImageAnimation
{
public:
  MovingImageAnimationWrapper() {}

  std::vector<uint8_t> UpdateFrame() { return MovingImageAnimation::UpdateFrame(); }
  void SetParam(const std::string & param) { MovingImageAnimation::SetParam(param); }
};

class TestMovingImageAnimation : public testing::Test
{
public:
  TestMovingImageAnimation();
  ~TestMovingImageAnimation();

protected:
  std::string test_image_path = testing::TempDir() + "/test_image.png";
  std::unique_ptr<MovingImageAnimationWrapper> animation_;
};

TestMovingImageAnimation::TestMovingImageAnimation()
{
  gil::rgba8_image_t image(100, 100);
  gil::write_view(test_image_path, gil::const_view(image), gil::png_tag());
  animation_ = std::make_unique<MovingImageAnimationWrapper>();
}

TestMovingImageAnimation::~TestMovingImageAnimation() { std::filesystem::remove(test_image_path); }

TEST_F(TestMovingImageAnimation, Initialize)
{
  YAML::Node animation_description = YAML::Load("{duration: 2.0}");

  // missing image in description
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  animation_description["image"] = this->test_image_path;
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));
}

TEST_F(TestMovingImageAnimation, UpdateFrame)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description =
    YAML::Load("{duration: 2.0, image: " + this->test_image_path + "}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));

  auto frame = animation_->UpdateFrame();
  EXPECT_EQ(num_led * 4, frame.size());
}

TEST_F(TestMovingImageAnimation, SetParamInvalidParam)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description =
    YAML::Load("{duration: 2.0, image: " + this->test_image_path + "}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));
  EXPECT_THROW(this->animation_->SetParam(""), std::runtime_error);
}

TEST_F(TestMovingImageAnimation, SetParamDefaultValue)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description = YAML::Load(
    "{duration: 2.0, image: " + this->test_image_path + ", default_image_position: 0.5}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));
  EXPECT_NO_THROW(this->animation_->SetParam(""));
}

TEST_F(TestMovingImageAnimation, SetParam)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description =
    YAML::Load("{duration: 2.0, image: " + this->test_image_path + "}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));
  EXPECT_NO_THROW(this->animation_->SetParam("0.5"));
}
