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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_

#include <cstdint>
#include <future>
#include <memory>
#include <string>

#include <gmock/gmock.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"

namespace husarion_ugv_hardware_interfaces_test
{

class MockDriver : public husarion_ugv_hardware_interfaces::DriverInterface
{
public:
  MockDriver()
  {
    ON_CALL(*this, Boot()).WillByDefault(::testing::Invoke([]() {
      std::promise<void> promise;
      promise.set_value();
      return promise.get_future();
    }));
  }

  MOCK_METHOD(std::future<void>, Boot, (), (override));
  MOCK_METHOD(bool, IsCANError, (), (const, override));
  MOCK_METHOD(bool, IsHeartbeatTimeout, (), (const, override));

  MOCK_METHOD(husarion_ugv_hardware_interfaces::DriverState, ReadState, (), (override));
  MOCK_METHOD(void, ResetScript, (), (override));
  MOCK_METHOD(void, TurnOnEStop, (), (override));
  MOCK_METHOD(void, TurnOffEStop, (), (override));

  std::shared_ptr<husarion_ugv_hardware_interfaces::MotorDriverInterface> GetMotorDriver(
    const husarion_ugv_hardware_interfaces::MotorNames name) override
  {
    return motor_drivers_.at(name);
  }

  void AddMotorDriver(
    const husarion_ugv_hardware_interfaces::MotorNames name,
    std::shared_ptr<husarion_ugv_hardware_interfaces::MotorDriverInterface> motor_driver) override
  {
    motor_drivers_.emplace(name, motor_driver);
  }

  using NiceMock = testing::NiceMock<MockDriver>;

private:
  std::map<
    husarion_ugv_hardware_interfaces::MotorNames,
    std::shared_ptr<husarion_ugv_hardware_interfaces::MotorDriverInterface>>
    motor_drivers_;
};

class MockMotorDriver : public husarion_ugv_hardware_interfaces::MotorDriverInterface
{
public:
  MOCK_METHOD(husarion_ugv_hardware_interfaces::MotorDriverState, ReadState, (), (override));
  MOCK_METHOD(void, SendCmdVel, (const std::int32_t cmd), (override));
  MOCK_METHOD(void, TurnOnSafetyStop, (), (override));

  using NiceMock = testing::NiceMock<MockMotorDriver>;
};

}  // namespace husarion_ugv_hardware_interfaces_test

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_MOCK_DRIVER_HPP_
