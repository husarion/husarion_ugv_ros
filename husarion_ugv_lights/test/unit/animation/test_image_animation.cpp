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
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_lights/animation/image_animation.hpp"

namespace gil = boost::gil;

class ImageAnimationWrapper : public husarion_ugv_lights::ImageAnimation
{
public:
  ImageAnimationWrapper() {}

  std::filesystem::path ParseImagePath(const std::string & image_path) const
  {
    return ImageAnimation::ParseImagePath(image_path);
  }

  gil::rgba8_image_t RGBAImageResize(
    const gil::rgba8_image_t & image, const std::size_t width, const std::size_t height)
  {
    return ImageAnimation::RGBAImageResize(image, width, height);
  }

  void RGBAImageConvertColor(gil::rgba8_image_t & image, const std::uint32_t color)
  {
    return ImageAnimation::RGBAImageConvertColor(image, color);
  }

  gil::gray_alpha8_image_t RGBAImageConvertToGrey(gil::rgba8_image_t & image)
  {
    return ImageAnimation::RGBAImageConvertToGrey(image);
  }

  void GreyImageNormalizeBrightness(gil::gray_alpha8_image_t & image)
  {
    return ImageAnimation::GreyImageNormalizeBrightness(image);
  }

  std::vector<uint8_t> UpdateFrame() { return ImageAnimation::UpdateFrame(); }
};

class TestImageAnimation : public testing::Test
{
public:
  TestImageAnimation();
  ~TestImageAnimation();

protected:
  std::string test_image_path = testing::TempDir() + "/test_image.png";
  std::unique_ptr<ImageAnimationWrapper> animation_;
};

TestImageAnimation::TestImageAnimation()
{
  gil::rgba8_image_t image(100, 100);
  gil::write_view(test_image_path, gil::const_view(image), gil::png_tag());
  animation_ = std::make_unique<ImageAnimationWrapper>();
}

TestImageAnimation::~TestImageAnimation() { std::filesystem::remove(test_image_path); }

TEST_F(TestImageAnimation, ParseImagePath)
{
  std::string image_path = "not/a/global/path";
  EXPECT_THROW(animation_->ParseImagePath(image_path), std::runtime_error);

  image_path = "/path/to/no/existing/file";
  EXPECT_THROW(animation_->ParseImagePath(image_path), std::runtime_error);

  // should return the same path if path is global and file exists
  EXPECT_EQ(this->test_image_path, animation_->ParseImagePath(this->test_image_path));

  // test ROS package path
  image_path = "$(find invalid_ros_package)/test/files/animation.png";
  EXPECT_THROW(animation_->ParseImagePath(image_path), std::runtime_error);

  // invalid substitution
  image_path = "$(fin husarion_ugv_lights)/test/files/animation.png";
  EXPECT_THROW(animation_->ParseImagePath(image_path), std::runtime_error);
  image_path = "$(find husarion_ugv_lights/test/files/animation.png";
  EXPECT_THROW(animation_->ParseImagePath(image_path), std::runtime_error);

  // following ones may not work if ROS package is not build or sourced
  image_path = "$(find husarion_ugv_lights)/test/files/animation.png";
  EXPECT_NO_THROW(animation_->ParseImagePath(image_path));

  // multiple spaces after find syntax
  image_path = "$(find    husarion_ugv_lights)/test/files/animation.png";
  EXPECT_NO_THROW(animation_->ParseImagePath(image_path));
}

TEST_F(TestImageAnimation, RGBAImageResize)
{
  gil::rgba8_image_t input_image(100, 100);
  auto output_image = animation_->RGBAImageResize(input_image, 75, 112);
  EXPECT_EQ(75, output_image.width());
  EXPECT_EQ(112, output_image.height());
}

TEST_F(TestImageAnimation, RGBAImageConvertColor)
{
  gil::rgba8_image_t white_image(5, 5);
  gil::fill_pixels(gil::view(white_image), gil::rgba8_pixel_t(255, 255, 255, 255));

  // copy white image
  auto blue_image = white_image;
  animation_->RGBAImageConvertColor(blue_image, 0x0000FF);
  // expect r,g pixels to be 0, b to be 255
  gil::for_each_pixel(gil::view(blue_image), [](const gil::rgba8_pixel_t & pixel) {
    EXPECT_EQ(0, pixel[0]);
    EXPECT_EQ(0, pixel[1]);
    EXPECT_EQ(255, pixel[2]);
    EXPECT_EQ(255, pixel[3]);
  });

  // copy white image
  auto green_image = white_image;
  animation_->RGBAImageConvertColor(green_image, 0x00FF00);
  // expect r,b pixels to be 0, g to be 255
  gil::for_each_pixel(gil::view(green_image), [](const gil::rgba8_pixel_t & pixel) {
    EXPECT_EQ(0, pixel[0]);
    EXPECT_EQ(255, pixel[1]);
    EXPECT_EQ(0, pixel[2]);
    EXPECT_EQ(255, pixel[3]);
  });

  // copy white image
  auto red_image = white_image;
  animation_->RGBAImageConvertColor(red_image, 0xFF0000);
  // expect g,b pixels to be 0, r to be 255
  gil::for_each_pixel(gil::view(red_image), [](const gil::rgba8_pixel_t & pixel) {
    EXPECT_EQ(255, pixel[0]);
    EXPECT_EQ(0, pixel[1]);
    EXPECT_EQ(0, pixel[2]);
    EXPECT_EQ(255, pixel[3]);
  });
}

TEST_F(TestImageAnimation, RGBAImageConvertToGrey)
{
  gil::rgba8_image_t rgb_image(5, 5);
  gil::fill_pixels(gil::view(rgb_image), gil::rgba8_pixel_t(30, 100, 200, 255));

  const int expected_grey_value = 0.299 * 30 + 0.587 * 100 + 0.114 * 200;
  auto grey_image = animation_->RGBAImageConvertToGrey(rgb_image);
  gil::for_each_pixel(
    gil::view(grey_image), [expected_grey_value](const gil::gray_alpha8_pixel_t & pixel) {
      EXPECT_EQ(expected_grey_value, pixel[0]);
    });
}

TEST_F(TestImageAnimation, GreyImageNormalizeBrightness)
{
  gil::gray_alpha8_image_t grey_image(5, 5);
  gil::fill_pixels(gil::view(grey_image), gil::gray_alpha8_pixel_t(204));
  // make the first row a bit darker
  for (int i = 0; i < grey_image.width(); i++) {
    gil::at_c<0>(*gil::view(grey_image).at(i)) = 102;
  }

  animation_->GreyImageNormalizeBrightness(grey_image);
  // check the first row
  for (int i = 0; i < grey_image.width(); i++) {
    EXPECT_EQ(int(102.0 / 204 * 255), gil::at_c<0>(*gil::view(grey_image).at(i)));
  }
  // check the rest of the image
  for (int i = grey_image.width(); i < grey_image.width() * grey_image.height(); i++) {
    EXPECT_EQ(255, gil::at_c<0>(*gil::view(grey_image).at(i)));
  }
}

TEST_F(TestImageAnimation, Initialize)
{
  YAML::Node animation_description = YAML::Load("{duration: 2.0}");

  // missing image in description
  EXPECT_THROW(animation_->Initialize(animation_description, 10, 10.0), std::runtime_error);

  animation_description["image"] = this->test_image_path;
  EXPECT_NO_THROW(animation_->Initialize(animation_description, 10, 10.0));
}

TEST_F(TestImageAnimation, UpdateFrame)
{
  const std::size_t num_led = 20;
  YAML::Node animation_description =
    YAML::Load("{duration: 2.0, image: " + this->test_image_path + "}");

  ASSERT_NO_THROW(animation_->Initialize(animation_description, num_led, 10.0));

  auto frame = animation_->UpdateFrame();
  EXPECT_EQ(num_led * 4, frame.size());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
