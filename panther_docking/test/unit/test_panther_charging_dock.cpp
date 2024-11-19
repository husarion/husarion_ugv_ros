// Copyright (c) 2024 Husarion Sp. z o.o.
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

#include <memory>

#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <panther_docking/panther_charging_dock.hpp>

static constexpr char kBaseFrame[] = "base_link";
static constexpr char kOdomFrame[] = "odom";

class PantherChargingDockWrapper : public panther_docking::PantherChargingDock
{
public:
  void setDockPose(geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    panther_docking::PantherChargingDock::setDockPose(msg);
  }

  void setWiboticInfo(wibotic_msgs::msg::WiboticInfo::SharedPtr msg)
  {
    panther_docking::PantherChargingDock::setWiboticInfo(msg);
  }
};

class TestPantherChargingDock : public ::testing::Test
{
protected:
  TestPantherChargingDock();
  void SetTransform(
    const std::string & frame_id, const std::string & child_frame_id,
    const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Transform & transform);

  void ActivateWiboticInfo();

  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::shared_ptr<PantherChargingDockWrapper> dock_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr dock_pose_pub;
  tf2_ros::Buffer::SharedPtr tf_buffer_;
};

TestPantherChargingDock::TestPantherChargingDock()
{
  node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_node");
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

  // Silence error about dedicated thread's being necessary
  tf_buffer_->setUsingDedicatedThread(true);

  dock_ = std::make_shared<PantherChargingDockWrapper>();
  dock_pose_pub = node_->create_publisher<geometry_msgs::msg::PoseStamped>("dock_pose", 10);

  node_->configure();
  node_->activate();
}

void TestPantherChargingDock::SetTransform(
  const std::string & frame_id, const std::string & child_frame_id,
  const builtin_interfaces::msg::Time & stamp, const geometry_msgs::msg::Transform & transform)
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = stamp;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.transform = transform;

  tf_buffer_->setTransform(transform_stamped, "unittest", true);
}

void TestPantherChargingDock::ActivateWiboticInfo()
{
  node_->declare_parameter("dock.use_wibotic_info", true);
  node_->declare_parameter("dock.wibotic_info_timeout", 1.0);
  dock_->configure(node_, "dock", tf_buffer_);
  dock_->activate();
}

TEST_F(TestPantherChargingDock, FailConfigureNoNode)
{
  node_.reset();
  ASSERT_THROW({ dock_->configure(node_, "dock", tf_buffer_); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, FailConfigureNoTfBuffer)
{
  tf_buffer_.reset();
  ASSERT_THROW({ dock_->configure(node_, "dock", tf_buffer_); }, std::runtime_error);
}

TEST_F(TestPantherChargingDock, GetStagingPoseLocal)
{
  dock_->configure(node_, "dock", tf_buffer_);
  dock_->activate();

  geometry_msgs::msg::PoseStamped::SharedPtr dock_pose =
    std::make_shared<geometry_msgs::msg::PoseStamped>();
  dock_pose->pose.position.x = 1.0;
  dock_pose->pose.position.y = 1.0;
  dock_pose->pose.position.z = 0.0;
  dock_pose->pose.orientation.w = 1.0;

  dock_->setDockPose(dock_pose);
  geometry_msgs::msg::PoseStamped pose;

  geometry_msgs::msg::PoseStamped staging_pose;
  ASSERT_THROW(
    { staging_pose = dock_->getStagingPose(pose.pose, kOdomFrame); },
    opennav_docking_core::FailedToDetectDock);

  dock_pose->header.frame_id = kOdomFrame;
  dock_->setDockPose(dock_pose);

  staging_pose = dock_->getStagingPose(pose.pose, kOdomFrame);

  ASSERT_FLOAT_EQ(staging_pose.pose.position.x, 0.3);
  ASSERT_FLOAT_EQ(staging_pose.pose.position.y, 1.0);
  ASSERT_FLOAT_EQ(staging_pose.pose.position.z, 0.0);
}

// TODO: fill after nav2 tests
// TEST_F(TestPantherChargingDock, GetStagingPoseGlobal){
// }

TEST_F(TestPantherChargingDock, GetRefinedPose)
{
  node_->declare_parameter("dock.external_detection_timeout", 0.5);
  dock_->configure(node_, "dock", tf_buffer_);
  dock_->activate();

  geometry_msgs::msg::PoseStamped::SharedPtr dock_pose =
    std::make_shared<geometry_msgs::msg::PoseStamped>();
  dock_pose->pose.position.x = 1.0;
  dock_pose->pose.position.y = 1.0;
  dock_pose->pose.position.z = 0.0;
  dock_pose->pose.orientation.w = 1.0;

  dock_->setDockPose(dock_pose);

  geometry_msgs::msg::PoseStamped pose;

  ASSERT_THROW({ dock_->getRefinedPose(pose); }, opennav_docking_core::FailedToDetectDock);

  dock_pose->header.frame_id = kOdomFrame;
  dock_->setDockPose(dock_pose);
  ASSERT_FALSE(dock_->getRefinedPose(pose));

  dock_pose->header.stamp = node_->now();
  dock_->setDockPose(dock_pose);
  ASSERT_TRUE(dock_->getRefinedPose(pose));

  ASSERT_FLOAT_EQ(pose.pose.position.x, 1.0);
  ASSERT_FLOAT_EQ(pose.pose.position.y, 1.0);
  ASSERT_FLOAT_EQ(pose.pose.position.z, 0.0);
}

TEST_F(TestPantherChargingDock, IsDocked)
{
  node_->declare_parameter("dock.external_detection_timeout", 0.5);
  dock_->configure(node_, "dock", tf_buffer_);
  dock_->activate();

  auto transform = geometry_msgs::msg::Transform();
  transform.translation.x = 1.0;
  transform.translation.y = 2.0;
  transform.translation.z = 3.0;

  SetTransform(kOdomFrame, kBaseFrame, node_->now(), transform);
  geometry_msgs::msg::PoseStamped::SharedPtr dock_pose =
    std::make_shared<geometry_msgs::msg::PoseStamped>();
  dock_pose->header.frame_id = kOdomFrame;
  dock_pose->header.stamp = node_->now();
  dock_pose->pose.position.x = transform.translation.x - 0.1;
  dock_pose->pose.position.y = transform.translation.y;
  dock_pose->pose.position.z = transform.translation.z;
  dock_->setDockPose(dock_pose);

  ASSERT_FALSE(dock_->isDocked());

  dock_pose->pose.position.x = transform.translation.x;
  dock_pose->pose.position.y = transform.translation.y;
  dock_pose->pose.position.z = transform.translation.z;
  // Set dock pose 10 times to ensure that filter stabilize the pose
  for (std::size_t i = 0; i < 10; i++) {
    dock_->setDockPose(dock_pose);
  }

  ASSERT_TRUE(dock_->isDocked());
}

TEST_F(TestPantherChargingDock, IsChargingNoWiboticInfo)
{
  ActivateWiboticInfo();
  ASSERT_THROW({ dock_->isCharging(); }, opennav_docking_core::FailedToCharge);
}

TEST_F(TestPantherChargingDock, IsChargingTimeout)
{
  ActivateWiboticInfo();

  wibotic_msgs::msg::WiboticInfo::SharedPtr wibotic_info =
    std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  dock_->setWiboticInfo(wibotic_info);
  ASSERT_FALSE(dock_->isCharging());
}

TEST_F(TestPantherChargingDock, IsChargingCurrentZero)
{
  ActivateWiboticInfo();
  wibotic_msgs::msg::WiboticInfo::SharedPtr wibotic_info =
    std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  wibotic_info->header.stamp = node_->now();
  wibotic_info->i_charger = 0.0;

  dock_->setWiboticInfo(wibotic_info);
  ASSERT_FALSE(dock_->isCharging());
}

TEST_F(TestPantherChargingDock, IsChargingTimeoutWithGoodCurrent)
{
  ActivateWiboticInfo();
  wibotic_msgs::msg::WiboticInfo::SharedPtr wibotic_info =
    std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  wibotic_info->i_charger = 0.1;

  dock_->setWiboticInfo(wibotic_info);
  ASSERT_FALSE(dock_->isCharging());
}

TEST_F(TestPantherChargingDock, IsChargingGoodCurrentWithoutTimeout)
{
  ActivateWiboticInfo();
  wibotic_msgs::msg::WiboticInfo::SharedPtr wibotic_info =
    std::make_shared<wibotic_msgs::msg::WiboticInfo>();
  wibotic_info->i_charger = 0.1;
  wibotic_info->header.stamp = node_->now();

  dock_->setWiboticInfo(wibotic_info);
  ASSERT_TRUE(dock_->isCharging());
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);

  auto run_tests = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return run_tests;
}
