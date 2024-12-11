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

#include <gtest/gtest.h>
#include <chrono>

#include <std_msgs/msg/header.hpp>

#include "husarion_ugv_utils/ros_utils.hpp"

using HeaderMsg = std_msgs::msg::Header;

TEST(TestVerifyTimestampGap, TimestampGapOk)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 8;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto max_timestamp_gap = std::chrono::seconds(2);
  EXPECT_NO_THROW(
    husarion_ugv_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap));
}

TEST(TestVerifyTimestampGap, TimestampGapExceeding)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 8;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto max_timestamp_gap = std::chrono::seconds(1);

  EXPECT_THROW(
    husarion_ugv_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap),
    std::runtime_error);
}

TEST(TestVerifyTimestampGap, TimestampNotSet)
{
  HeaderMsg header_1;
  HeaderMsg header_2;

  auto max_timestamp_gap = std::chrono::seconds(1);

  EXPECT_THROW(
    husarion_ugv_utils::ros::VerifyTimestampGap(header_1, header_2, max_timestamp_gap),
    std::runtime_error);
}

TEST(TestMergeHeaders, SameFrameIds)
{
  HeaderMsg header_1;
  header_1.frame_id = "frame";

  HeaderMsg header_2;
  header_2.frame_id = "frame";

  HeaderMsg merged_header;

  EXPECT_NO_THROW(merged_header = husarion_ugv_utils::ros::MergeHeaders(header_1, header_2));
  EXPECT_STREQ("frame", merged_header.frame_id.c_str());
}

TEST(TestMergeHeaders, DifferentFrameIds)
{
  HeaderMsg header_1;
  header_1.frame_id = "frame_1";

  HeaderMsg header_2;
  header_2.frame_id = "frame_2";

  EXPECT_THROW(husarion_ugv_utils::ros::MergeHeaders(header_1, header_2), std::runtime_error);
}

TEST(TestMergeHeaders, EarlierTimestampNanosec)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 10;
  header_1.stamp.nanosec = 200000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = husarion_ugv_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 10);
  EXPECT_EQ(merged_header.stamp.nanosec, 200000000);
}

TEST(TestMergeHeaders, EarlierTimestampSec)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 9;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 10;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = husarion_ugv_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 9);
  EXPECT_EQ(merged_header.stamp.nanosec, 500000000);
}

TEST(TestMergeHeaders, LaterTimestamp)
{
  HeaderMsg header_1;
  header_1.stamp.sec = 10;
  header_1.stamp.nanosec = 500000000;

  HeaderMsg header_2;
  header_2.stamp.sec = 9;
  header_2.stamp.nanosec = 500000000;

  auto merged_header = husarion_ugv_utils::ros::MergeHeaders(header_1, header_2);

  EXPECT_EQ(merged_header.stamp.sec, 9);
  EXPECT_EQ(merged_header.stamp.nanosec, 500000000);
}

TEST(TestAddNamespaceToFrameID, WithoutNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "";

  auto result = husarion_ugv_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "frame");
}

TEST(TestAddNamespaceToFrameID, WithSlashNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "/";

  auto result = husarion_ugv_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "frame");
}

TEST(TestAddNamespaceToFrameID, WithSlashedNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "/namespace";

  auto result = husarion_ugv_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "namespace/frame");
}

TEST(TestAddNamespaceToFrameID, WithNamespace)
{
  std::string frame_id = "frame";
  std::string ns = "namespace";

  auto result = husarion_ugv_utils::ros::AddNamespaceToFrameID(frame_id, ns);
  EXPECT_EQ(result, "namespace/frame");
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
