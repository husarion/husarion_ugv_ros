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
#include <cmath>

#include <gtest/gtest.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "panther_utils/tf2_utils.hpp"

static constexpr char kBaseFrame[] = "base_link";
static constexpr char kOdomFrame[] = "odom";

class TestTransformPose : public ::testing::Test
{
public:
  TestTransformPose();

  void SetBaseLinkToOdomTransform(const builtin_interfaces::msg::Time & stamp);

protected:
  rclcpp::Clock::SharedPtr clock_;
  tf2_ros::Buffer::SharedPtr tf2_buffer_;

  geometry_msgs::msg::TransformStamped transform_;
};

TestTransformPose::TestTransformPose()
{
  clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(clock_);

  // Silence error about dedicated thread's being necessary
  tf2_buffer_->setUsingDedicatedThread(true);

  transform_.header.frame_id = kOdomFrame;
  transform_.child_frame_id = kBaseFrame;
  transform_.transform.translation.x = 0.3;
  transform_.transform.translation.y = 0.2;
  transform_.transform.translation.z = 0.1;
  transform_.transform.rotation.x = 0.0;
  transform_.transform.rotation.y = 0.0;
  transform_.transform.rotation.z = 0.0;
  transform_.transform.rotation.w = 1.0;
}

void TestTransformPose::SetBaseLinkToOdomTransform(const builtin_interfaces::msg::Time & stamp)
{
  transform_.header.stamp = stamp;
  tf2_buffer_->setTransform(transform_, "unittest", false);
}

TEST_F(TestTransformPose, EmptyFrame)
{
  geometry_msgs::msg::PoseStamped pose;
  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, ""); }, std::runtime_error);
}

TEST_F(TestTransformPose, TransformMissing)
{
  geometry_msgs::msg::PoseStamped pose;

  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, kBaseFrame); },
    std::runtime_error);
  pose.header.frame_id = kOdomFrame;
  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, kBaseFrame); },
    std::runtime_error);
}

TEST_F(TestTransformPose, TransformWithinSameFrame)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kOdomFrame;
  pose.pose.position.x = 0.1;
  pose.header.stamp = clock_->now();

  ASSERT_NO_THROW(
    { pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, kOdomFrame, 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, 0.1, 0.01);
  EXPECT_NEAR(pose.pose.position.y, 0.0, 0.01);
  EXPECT_NEAR(pose.pose.position.z, 0.0, 0.01);
  EXPECT_EQ(pose.header.frame_id, kOdomFrame);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, TransformToDifferentFrame)
{
  geometry_msgs::msg::PoseStamped pose;
  const auto stamp = clock_->now();
  pose.header.frame_id = kOdomFrame;
  pose.pose.position.x = 0.1;
  pose.header.stamp = stamp;
  SetBaseLinkToOdomTransform(stamp);

  ASSERT_NO_THROW(
    { pose = panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, kBaseFrame, 10.0); };);
  EXPECT_NEAR(pose.pose.position.x, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.y, -0.2, 0.01);
  EXPECT_NEAR(pose.pose.position.z, -0.1, 0.01);
  EXPECT_EQ(pose.header.frame_id, kBaseFrame);
  EXPECT_NEAR(tf2::getYaw(pose.pose.orientation), 0.0, 0.01);
}

TEST_F(TestTransformPose, TestTimeout)
{
  const auto stamp = clock_->now();
  SetBaseLinkToOdomTransform(stamp);
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = kOdomFrame;
  pose.header.stamp = stamp;
  pose.header.stamp.sec += 5;

  EXPECT_THROW(
    { panther_utils::tf2_utils::TransformPose(tf2_buffer_, pose, kBaseFrame, 1.0); },
    std::runtime_error);
}

TEST(OffsetPose, CheckDefaultPose)
{
  const auto translation = tf2::Vector3(0.1, 0.2, 0.3);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, 1.57, 3.14);

  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);

  geometry_msgs::msg::PoseStamped pose;
  auto offset_pose = panther_utils::tf2_utils::OffsetPose(pose, transform);

  EXPECT_NEAR(offset_pose.pose.position.x, translation.getX(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.y, translation.getY(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.z, translation.getZ(), 0.01);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(offset_pose.pose.orientation, offset_rotation);
  tf2::Matrix3x3 matrix(offset_rotation);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 0.01);
  EXPECT_NEAR(pitch, 1.57, 0.01);
  EXPECT_NEAR(yaw, 3.14, 0.01);
}

TEST(OffsetPose, CheckDefinedPose)
{
  const auto translation = tf2::Vector3(0.7, 0.2, 0.3);
  tf2::Quaternion rotation;
  rotation.setRPY(0.0, M_PI / 2.0, M_PI);

  tf2::Transform transform;
  transform.setOrigin(translation);
  transform.setRotation(rotation);

  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = 0.1;
  pose.pose.position.y = 0.9;
  pose.pose.position.z = 0.2;
  tf2::Quaternion pose_rotation;
  pose_rotation.setRPY(M_PI, M_PI / 2.0, 0.0);
  pose.pose.orientation = tf2::toMsg(pose_rotation);

  auto offset_pose = panther_utils::tf2_utils::OffsetPose(pose, transform);

  EXPECT_NEAR(offset_pose.pose.position.x, pose.pose.position.x - translation.getZ(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.y, pose.pose.position.y - translation.getY(), 0.01);
  EXPECT_NEAR(offset_pose.pose.position.z, pose.pose.position.z - translation.getX(), 0.01);

  tf2::Quaternion offset_rotation;
  tf2::fromMsg(offset_pose.pose.orientation, offset_rotation);
  tf2::Matrix3x3 matrix(offset_rotation);
  double roll, pitch, yaw;
  matrix.getRPY(roll, pitch, yaw);
  EXPECT_NEAR(roll, 0.0, 0.01);
  EXPECT_NEAR(pitch, 0.0, 0.01);
  EXPECT_NEAR(yaw, 0.0, 0.01);
}

TEST(TestArePosesNear, EmptyFrame)
{
  geometry_msgs::msg::PoseStamped pose_1;
  geometry_msgs::msg::PoseStamped pose_2;

  EXPECT_THROW(
    { panther_utils::tf2_utils::ArePosesNear(pose_1, pose_2, 0.1, 0.1); }, std::runtime_error);
}

TEST(TestArePosesNear, DifferentFrames)
{
  geometry_msgs::msg::PoseStamped pose_1;
  pose_1.header.frame_id = kBaseFrame;
  geometry_msgs::msg::PoseStamped pose_2;
  pose_2.header.frame_id = kOdomFrame;

  EXPECT_THROW(
    { panther_utils::tf2_utils::ArePosesNear(pose_1, pose_2, 0.1, 0.1); }, std::runtime_error);
}

TEST(TestArePosesNear, TooFarDistance)
{
  geometry_msgs::msg::PoseStamped pose_1;
  pose_1.header.frame_id = kBaseFrame;
  pose_1.pose.position.x = 0.1;
  pose_1.pose.position.y = 0.1;
  pose_1.pose.position.z = 0.0;

  geometry_msgs::msg::PoseStamped pose_2;
  pose_2.header.frame_id = kBaseFrame;
  pose_2.pose.position.x = 0.2;
  pose_2.pose.position.y = 0.2;
  pose_2.pose.position.z = 0.0;

  EXPECT_FALSE(panther_utils::tf2_utils::ArePosesNear(pose_1, pose_2, 0.05, 0.05));
}

TEST(TestArePosesNear, TooFarAngle)
{
  geometry_msgs::msg::PoseStamped pose_1;
  pose_1.header.frame_id = kBaseFrame;
  tf2::Quaternion pose_1_rotation;
  pose_1_rotation.setRPY(0.0, 0.0, 0.1);
  pose_1.pose.orientation = tf2::toMsg(pose_1_rotation);

  geometry_msgs::msg::PoseStamped pose_2;
  pose_2.header.frame_id = kBaseFrame;
  tf2::Quaternion pose_2_rotation;
  pose_2_rotation.setRPY(0.0, 0.0, 0.2);
  pose_2.pose.orientation = tf2::toMsg(pose_2_rotation);

  EXPECT_FALSE(panther_utils::tf2_utils::ArePosesNear(pose_1, pose_2, 0.1, 0.05));
}

TEST(TestArePosesNear, NearAngleAndDistance)
{
  geometry_msgs::msg::PoseStamped pose_1;
  pose_1.header.frame_id = kBaseFrame;
  pose_1.pose.position.x = 0.1;
  pose_1.pose.position.y = 0.1;
  pose_1.pose.position.z = 0.0;
  tf2::Quaternion pose_1_rotation;
  pose_1_rotation.setRPY(0.0, 0.0, 0.1);
  pose_1.pose.orientation = tf2::toMsg(pose_1_rotation);

  geometry_msgs::msg::PoseStamped pose_2;
  pose_2.header.frame_id = kBaseFrame;
  pose_2.pose.position.x = 0.2;
  pose_2.pose.position.y = 0.2;
  pose_2.pose.position.z = 0.0;
  tf2::Quaternion pose_2_rotation;
  pose_2_rotation.setRPY(0.0, 0.0, 0.1);
  pose_2.pose.orientation = tf2::toMsg(pose_2_rotation);

  EXPECT_TRUE(panther_utils::tf2_utils::ArePosesNear(pose_1, pose_2, 0.2, 0.2));
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
