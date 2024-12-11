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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_

#include <memory>
#include <unordered_map>
#include <vector>

#include "husarion_ugv_lights/led_components/led_panel.hpp"
#include "husarion_ugv_lights/led_components/led_segment.hpp"

namespace husarion_ugv_lights
{

class SegmentConverter
{
public:
  SegmentConverter() = default;
  ~SegmentConverter() = default;

  void Convert(
    const std::unordered_map<std::string, std::shared_ptr<LEDSegment>> & segments,
    const std::unordered_map<std::size_t, std::shared_ptr<LEDPanel>> & panels);
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_SEGMENT_CONVERTER_HPP_
