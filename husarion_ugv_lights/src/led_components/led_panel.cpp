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

#include "husarion_ugv_lights/led_components/led_panel.hpp"

#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace husarion_ugv_lights
{

LEDPanel::LEDPanel(const std::size_t num_led) : num_led_(num_led)
{
  frame_ = std::vector<std::uint8_t>(num_led_ * 4, 0);
}

void LEDPanel::UpdateFrame(
  const std::size_t iterator_first, const std::vector<std::uint8_t> & values)
{
  if (values.empty()) {
    throw std::runtime_error("The input values vector is empty.");
  }
  if (values.size() > frame_.size()) {
    throw std::runtime_error(
      "The size of the input values (" + std::to_string(values.size()) +
      ") exceeds the size of the frame (" + std::to_string(frame_.size()) + ").");
  }
  if (values.size() + iterator_first > frame_.size()) {
    throw std::runtime_error(
      "The input values vector can't fit into the frame at the given "
      "position (" +
      std::to_string(iterator_first) + "). The size of the values vector is " +
      std::to_string(values.size()) + ", but the remaining space in the frame is " +
      std::to_string(frame_.size() - iterator_first) + ".");
  }

  std::copy(values.begin(), values.end(), frame_.begin() + iterator_first);
}

}  // namespace husarion_ugv_lights
