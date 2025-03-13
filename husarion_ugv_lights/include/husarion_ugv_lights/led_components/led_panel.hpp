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

#ifndef HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_PANEL_HPP_
#define HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_PANEL_HPP_

#include <cstdint>
#include <vector>

namespace husarion_ugv_lights
{

/**
 * @brief Class that represents LED panel of the robot
 */
class LEDPanel
{
public:
  LEDPanel(const std::size_t num_led);

  ~LEDPanel() = default;

  /**
   * @brief Updates LED panel frame
   *
   * @param iterator_first position at which values will be inserted
   * @param values vector with values that will be inserted into the frame
   *
   * @exception std::runtime_error if values vector is empty or can't be fit into the farme
   */
  void UpdateFrame(const std::size_t iterator_first, const std::vector<std::uint8_t> & values);

  std::vector<std::uint8_t> GetFrame() const { return frame_; }
  std::size_t GetNumberOfLeds() const { return num_led_; }

private:
  const std::size_t num_led_;
  std::vector<std::uint8_t> frame_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_LED_COMPONENTS_LED_PANEL_HPP_
