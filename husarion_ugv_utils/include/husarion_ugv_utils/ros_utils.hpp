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

#ifndef HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_ROS_UTILS_HPP_
#define HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_ROS_UTILS_HPP_

#include <chrono>

#include <std_msgs/msg/header.hpp>

namespace husarion_ugv_utils::ros
{

/**
 * Verifies the timestamp gap between two headers and throws an exception if the gap exceeds the
 * maximum allowed gap.
 *
 * @param header_1 The first header containing the timestamp.
 * @param header_2 The second header containing the timestamp.
 * @param max_timestamp_gap The maximum allowed timestamp gap in seconds.
 * @throws `std::runtime_error` If either of the timestamps is not set or if the timestamp gap
 * exceeds the maximum allowed gap.
 */
void VerifyTimestampGap(
  const std_msgs::msg::Header & header_1, const std_msgs::msg::Header & header_2,
  std::chrono::seconds max_timestamp_gap)
{
  const auto timestamp_1_ns = std::chrono::seconds(header_1.stamp.sec) +
                              std::chrono::nanoseconds(header_1.stamp.nanosec);
  const auto timestamp_2_ns = std::chrono::seconds(header_2.stamp.sec) +
                              std::chrono::nanoseconds(header_2.stamp.nanosec);

  if (timestamp_1_ns.count() == 0 || timestamp_2_ns.count() == 0) {
    throw std::runtime_error("Timestamps are not set.");
  }

  auto timestamp_gap = std::abs(
    std::chrono::duration_cast<std::chrono::seconds>(timestamp_1_ns - timestamp_2_ns).count());

  if (timestamp_gap > max_timestamp_gap.count()) {
    throw std::runtime_error("Timestamp gap exceeds the maximum allowed gap.");
  }
}

/**
 * @brief Merges two headers into a single header.
 *
 * This function takes two headers as input and merges them into a single header. The merged header
 * will have the same frame ID as the input headers. The timestamp of the merged header will be the
 * earliest timestamp between the two input headers.
 *
 * If the frame IDs of the input headers are different, an exception of type std::runtime_error
 * will be thrown.
 *
 * @param header_1 The first header to be merged.
 * @param header_2 The second header to be merged.
 * @return The merged header.
 * @throws `std::runtime_error` If the frame IDs of the input headers are different.
 */
std_msgs::msg::Header MergeHeaders(
  const std_msgs::msg::Header & header_1, const std_msgs::msg::Header & header_2)
{
  std_msgs::msg::Header merged_header;

  if (header_1.frame_id != header_2.frame_id) {
    throw std::runtime_error("Provided frame IDs are different, can't merge headers.");
  }

  merged_header.frame_id = header_1.frame_id;

  if (
    header_1.stamp.sec < header_2.stamp.sec ||
    (header_1.stamp.sec == header_2.stamp.sec && header_1.stamp.nanosec < header_2.stamp.nanosec)) {
    merged_header.stamp = header_1.stamp;
  } else {
    merged_header.stamp = header_2.stamp;
  }

  return merged_header;
}

/**
 * @brief Adds a namespace to a frame ID.
 *
 * This function adds a namespace to a frame ID. The namespace is added as a prefix to the frame ID.
 *
 * @param frame_id The frame ID to which the namespace should be added.
 * @param node_namespace The namespace to be added to the frame ID.
 *
 * @return The frame ID with the namespace added as a prefix.
 */
std::string AddNamespaceToFrameID(const std::string & frame_id, const std::string & node_namespace)
{
  std::string tf_prefix = node_namespace;

  if (tf_prefix != "/") {
    tf_prefix += '/';
  }

  if (tf_prefix.front() == '/') {
    tf_prefix.erase(0, 1);
  }

  return tf_prefix + frame_id;
}

}  // namespace husarion_ugv_utils::ros

#endif  // HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_ROS_UTILS_HPP_
