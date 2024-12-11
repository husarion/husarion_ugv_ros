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

#include "husarion_ugv_lights/led_components/led_animations_queue.hpp"
#include "husarion_ugv_lights/led_components/led_segment.hpp"

class TestLEDAnimationsQueue : public testing::Test
{
public:
  TestLEDAnimationsQueue();
  ~TestLEDAnimationsQueue() {}

  husarion_ugv_lights::LEDAnimation CreateLEDAnimation(
    const std::string & name, const std::uint8_t priority,
    const rclcpp::Time & init_time = rclcpp::Time(0));

protected:
  std::shared_ptr<husarion_ugv_lights::LEDAnimationsQueue> led_anim_queue_;
  std::unordered_map<std::string, std::shared_ptr<husarion_ugv_lights::LEDSegment>> segments_;

  const std::size_t max_queue_size_ = 5;
};

TestLEDAnimationsQueue::TestLEDAnimationsQueue()
{
  auto segment_1_desc = YAML::Load("{channel: 1, led_range: 0-10}");
  auto segment_2_desc = YAML::Load("{channel: 2, led_range: 0-10}");
  segments_.emplace(
    "segment_1", std::make_shared<husarion_ugv_lights::LEDSegment>(segment_1_desc, 50.0));
  segments_.emplace(
    "segment_2", std::make_shared<husarion_ugv_lights::LEDSegment>(segment_2_desc, 50.0));

  led_anim_queue_ = std::make_shared<husarion_ugv_lights::LEDAnimationsQueue>(5);
}

husarion_ugv_lights::LEDAnimation TestLEDAnimationsQueue::CreateLEDAnimation(
  const std::string & name, const std::uint8_t priority, const rclcpp::Time & init_time)
{
  husarion_ugv_lights::AnimationDescription anim_desc;
  anim_desc.segments = {"segment_1", "segment_2"};
  anim_desc.type = "husarion_ugv_lights::ImageAnimation";
  anim_desc.animation =
    YAML::Load("{image: $(find husarion_ugv_lights)/test/files/animation.png, duration: 2.0}");

  husarion_ugv_lights::LEDAnimationDescription led_anim_desc;
  led_anim_desc.id = 0;
  led_anim_desc.name = name;
  led_anim_desc.priority = priority;
  led_anim_desc.timeout = 10.0;
  led_anim_desc.animations = {anim_desc};

  return husarion_ugv_lights::LEDAnimation(led_anim_desc, segments_, init_time);
}

TEST_F(TestLEDAnimationsQueue, Put)
{
  auto led_anim =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 1));
  led_anim_queue_->Put(led_anim, rclcpp::Time(0));

  EXPECT_FALSE(led_anim_queue_->Empty());
  EXPECT_TRUE(led_anim_queue_->HasAnimation(led_anim));
}

TEST_F(TestLEDAnimationsQueue, PutQueueOverloaded)
{
  auto led_anim =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 1));
  for (std::size_t i = 0; i < max_queue_size_; i++) {
    led_anim_queue_->Put(led_anim, rclcpp::Time(0));
  }

  EXPECT_THROW(led_anim_queue_->Put(led_anim, rclcpp::Time(0)), std::runtime_error);
}

TEST_F(TestLEDAnimationsQueue, PutClearWhenPriorityEqualOne)
{
  auto led_anim_pr_1 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 1));
  auto led_anim_pr_2 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 2));
  auto led_anim_pr_3 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 3));

  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_1, rclcpp::Time(0));

  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_pr_1);
  // animation with priority 1 should remove animations with lower priorities
  EXPECT_TRUE(led_anim_queue_->Empty());
}

TEST_F(TestLEDAnimationsQueue, PutSortByPriority)
{
  auto led_anim_pr_2 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 2));
  auto led_anim_pr_3 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST", 3));

  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));

  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_pr_2);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_pr_2);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_pr_3);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_pr_3);
}

TEST_F(TestLEDAnimationsQueue, PutSortByTime)
{
  auto led_anim_t0 = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    CreateLEDAnimation("TEST", 1, rclcpp::Time(0)));
  auto led_anim_t1 = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    CreateLEDAnimation("TEST", 1, rclcpp::Time(1)));
  auto led_anim_t2 = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    CreateLEDAnimation("TEST", 1, rclcpp::Time(2)));
  auto led_anim_t3 = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    CreateLEDAnimation("TEST", 1, rclcpp::Time(3)));

  led_anim_queue_->Put(led_anim_t3, rclcpp::Time(4));
  led_anim_queue_->Put(led_anim_t1, rclcpp::Time(4));
  led_anim_queue_->Put(led_anim_t2, rclcpp::Time(4));
  led_anim_queue_->Put(led_anim_t0, rclcpp::Time(4));

  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_t0);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_t1);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_t2);
  EXPECT_TRUE(led_anim_queue_->Get() == led_anim_t3);
}

TEST_F(TestLEDAnimationsQueue, GetQueueEmpty)
{
  EXPECT_THROW(led_anim_queue_->Get(), std::runtime_error);
}

TEST_F(TestLEDAnimationsQueue, Clear)
{
  auto led_anim_pr_1 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST1", 1));
  auto led_anim_pr_2 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST2", 2));
  auto led_anim_pr_3 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST3", 3));

  led_anim_queue_->Put(led_anim_pr_1, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));

  led_anim_queue_->Clear(3);
  EXPECT_TRUE(led_anim_queue_->HasAnimation(led_anim_pr_1));
  EXPECT_TRUE(led_anim_queue_->HasAnimation(led_anim_pr_2));
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_3));

  led_anim_queue_->Clear();
  EXPECT_TRUE(led_anim_queue_->HasAnimation(led_anim_pr_1));
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_2));

  led_anim_queue_->Clear(1);
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_1));
  EXPECT_TRUE(led_anim_queue_->Empty());
}

TEST_F(TestLEDAnimationsQueue, ValidateAnimationTimedOut)
{
  auto led_anim = std::make_shared<husarion_ugv_lights::LEDAnimation>(
    CreateLEDAnimation("TEST", 1, rclcpp::Time(0)));

  led_anim_queue_->Put(led_anim, rclcpp::Time(0));
  led_anim_queue_->Validate(rclcpp::Time(0));
  EXPECT_TRUE(led_anim_queue_->HasAnimation(led_anim));

  // exceed timeout
  led_anim_queue_->Validate(rclcpp::Time(12, 0));
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim));
  EXPECT_TRUE(led_anim_queue_->Empty());
}

TEST_F(TestLEDAnimationsQueue, GetFirstAnimationPriority)
{
  auto led_anim_pr_1 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST1", 1));
  auto led_anim_pr_2 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST2", 2));
  auto led_anim_pr_3 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST3", 3));

  led_anim_queue_->Put(led_anim_pr_1, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));

  EXPECT_TRUE(led_anim_queue_->GetFirstAnimationPriority() == 1);
  led_anim_queue_->Get();
  EXPECT_TRUE(led_anim_queue_->GetFirstAnimationPriority() == 2);
  led_anim_queue_->Get();
  EXPECT_TRUE(led_anim_queue_->GetFirstAnimationPriority() == 3);
  led_anim_queue_->Get();
  EXPECT_TRUE(led_anim_queue_->Empty());
  EXPECT_TRUE(led_anim_queue_->GetFirstAnimationPriority() == 3);
}

TEST_F(TestLEDAnimationsQueue, Remove)
{
  auto led_anim_pr_1 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST1", 1));
  auto led_anim_pr_2 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST2", 2));
  auto led_anim_pr_3 =
    std::make_shared<husarion_ugv_lights::LEDAnimation>(CreateLEDAnimation("TEST3", 3));

  led_anim_queue_->Put(led_anim_pr_1, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_2, rclcpp::Time(0));
  led_anim_queue_->Put(led_anim_pr_3, rclcpp::Time(0));

  led_anim_queue_->Remove(led_anim_pr_2);
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_2));
  led_anim_queue_->Remove(led_anim_pr_3);
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_3));
  led_anim_queue_->Remove(led_anim_pr_1);
  EXPECT_FALSE(led_anim_queue_->HasAnimation(led_anim_pr_1));
  EXPECT_TRUE(led_anim_queue_->Empty());
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
