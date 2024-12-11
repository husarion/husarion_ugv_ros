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

#include "husarion_ugv_lights/led_components/led_animations_queue.hpp"

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "rclcpp/logging.hpp"
#include "rclcpp/time.hpp"

namespace husarion_ugv_lights
{

LEDAnimation::LEDAnimation(
  const LEDAnimationDescription & led_animation_description,
  const std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments,
  const rclcpp::Time & init_time)
: led_animation_description_(led_animation_description),
  init_time_(init_time),
  repeating_(false),
  param_("")
{
  for (const auto & animation : led_animation_description_.animations) {
    for (const auto & segment : animation.segments) {
      if (segments.find(segment) == segments.end()) {
        throw std::runtime_error("No segment with name: " + segment + ".");
      }
      animation_segments_.push_back(segments.at(segment));
    }
  }
}

bool LEDAnimation::IsFinished()
{
  return std::all_of(
    animation_segments_.begin(), animation_segments_.end(),
    [](const std::shared_ptr<LEDSegment> & segment) { return segment->IsAnimationFinished(); });
}

float LEDAnimation::GetProgress() const
{
  float progress = 1.0f;
  std::for_each(
    animation_segments_.begin(), animation_segments_.end(),
    [&progress](const std::shared_ptr<LEDSegment> & segment) {
      auto anim_progress = segment->GetAnimationProgress();
      progress = anim_progress < progress ? anim_progress : progress;
    });

  return progress;
}

void LEDAnimation::Reset(const rclcpp::Time & time)
{
  std::for_each(
    animation_segments_.begin(), animation_segments_.end(),
    [](const std::shared_ptr<LEDSegment> & segment) { segment->ResetAnimation(); });
  init_time_ = time;
}

void LEDAnimationsQueue::Put(
  const std::shared_ptr<LEDAnimation> & animation, const rclcpp::Time & time)
{
  if (animation->GetPriority() == 1) {
    Clear();
  }
  Validate(time);

  if (queue_.size() == max_queue_size_) {
    throw std::runtime_error("Animation queue overloaded.");
  }

  queue_.push_back(animation);
  std::sort(
    queue_.begin(), queue_.end(),
    [](const std::shared_ptr<LEDAnimation> & a, const std::shared_ptr<LEDAnimation> & b) {
      return a->GetPriority() < b->GetPriority() ||
             (a->GetPriority() == b->GetPriority() && a->GetInitTime() < b->GetInitTime());
    });
}

std::shared_ptr<LEDAnimation> LEDAnimationsQueue::Get()
{
  if (!queue_.empty()) {
    const auto animation = queue_.front();
    queue_.pop_front();
    return animation;
  }
  throw std::runtime_error("Queue empty.");
}

void LEDAnimationsQueue::Clear(const std::size_t priority)
{
  const auto new_end = std::remove_if(
    queue_.begin(), queue_.end(), [priority](const std::shared_ptr<LEDAnimation> & animation) {
      return animation->GetPriority() >= priority;
    });
  queue_.erase(new_end, queue_.end());
}

void LEDAnimationsQueue::Validate(const rclcpp::Time & time)
{
  for (auto it = queue_.begin(); it != queue_.end();) {
    if ((time - it->get()->GetInitTime()).seconds() > it->get()->GetTimeout()) {
      RCLCPP_WARN_STREAM(
        rclcpp::get_logger("lights_controller"),
        "Timeout for animation: " << it->get()->GetName() << ". Removing from the queue.");
      it = queue_.erase(it);
    } else {
      ++it;
    }
  }
}

std::size_t LEDAnimationsQueue::GetFirstAnimationPriority() const
{
  if (!Empty()) {
    return queue_.front()->GetPriority();
  }
  return LEDAnimation::kDefaultPriority;
}

void LEDAnimationsQueue::Remove(const std::shared_ptr<LEDAnimation> & animation)
{
  auto it = std::find(queue_.begin(), queue_.end(), animation);
  if (it != queue_.end()) {
    queue_.erase(it);
  }
}

bool LEDAnimationsQueue::HasAnimation(const std::shared_ptr<LEDAnimation> & animation) const
{
  return std::find(queue_.begin(), queue_.end(), animation) != queue_.end();
}

}  // namespace husarion_ugv_lights
