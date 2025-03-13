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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_

#include <map>
#include <string>

#include "gpiod.hpp"

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Enumeration representing available GPIO pins in the Panther system.
 */
enum class GPIOPin {
  AUX_PW_EN,
  CHRG_DISABLE,
  CHRG_SENSE,
  DRIVER_EN,
  E_STOP_RESET,
  FAN_SW,
  GPOUT1,
  GPOUT2,
  GPIN1,
  GPIN2,
  LED_SBC_SEL,
  SHDN_INIT,
  STAGE2_INPUT,
  VDIG_OFF,
  VMOT_ON,
  MOTOR_ON,
  WATCHDOG
};

/**
 * @brief Mapping of GPIO pins to their respective names.
 */
const std::map<GPIOPin, std::string> pin_names_{
  {GPIOPin::WATCHDOG, "WATCHDOG"},
  {GPIOPin::AUX_PW_EN, "AUX_PW_EN"},
  {GPIOPin::CHRG_DISABLE, "CHRG_DISABLE"},
  {GPIOPin::CHRG_SENSE, "CHRG_SENSE"},
  {GPIOPin::DRIVER_EN, "DRIVER_EN"},
  {GPIOPin::E_STOP_RESET, "E_STOP_RESET"},
  {GPIOPin::FAN_SW, "FAN_SW"},
  {GPIOPin::GPOUT1, "GPOUT1"},
  {GPIOPin::GPOUT2, "GPOUT2"},
  {GPIOPin::GPIN1, "GPIN1"},
  {GPIOPin::GPIN2, "GPIN2"},
  {GPIOPin::LED_SBC_SEL, "LED_SBC_SEL"},
  {GPIOPin::SHDN_INIT, "SHDN_INIT"},
  {GPIOPin::STAGE2_INPUT, "STAGE2_INPUT"},
  {GPIOPin::VDIG_OFF, "VDIG_OFF"},
  {GPIOPin::VMOT_ON, "VMOT_ON"},
  {GPIOPin::MOTOR_ON, "MOTOR_ON"},
};

/**
 * @brief Structure containing information related to GPIO pins such as pin configuration,
 * direction, value, etc. This information is required during the initialization process.
 */
struct GPIOInfo
{
  GPIOPin pin;
  gpiod::line::direction direction;
  bool active_low = false;
  gpiod::line::value init_value = gpiod::line::value::INACTIVE;
  gpiod::line::value value = gpiod::line::value::INACTIVE;
  gpiod::line::offset offset = -1;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_GPIO_TYPES_HPP_
