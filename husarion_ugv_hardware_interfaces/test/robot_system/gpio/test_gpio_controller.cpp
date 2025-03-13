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

#include <cstdlib>
#include <functional>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <gpiod.hpp>

#include <husarion_ugv_hardware_interfaces/robot_system/gpio/gpio_controller.hpp>

using GPIOInfo = husarion_ugv_hardware_interfaces::GPIOInfo;
using GPIOPin = husarion_ugv_hardware_interfaces::GPIOPin;

class MockGPIODriver : public husarion_ugv_hardware_interfaces::GPIODriverInterface
{
public:
  MOCK_METHOD(
    void, GPIOMonitorEnable, (const bool use_rt, const unsigned gpio_monit_thread_sched_priority),
    (override));
  MOCK_METHOD(
    void, ConfigureEdgeEventCallback, (const std::function<void(const GPIOInfo &)> & callback),
    (override));
  MOCK_METHOD(
    void, ChangePinDirection, (const GPIOPin pin, const gpiod::line::direction direction),
    (override));
  MOCK_METHOD(bool, IsPinAvailable, (const GPIOPin pin), (const, override));
  MOCK_METHOD(bool, IsPinActive, (const GPIOPin pin), (override));
  MOCK_METHOD(
    bool, SetPinValue,
    (const GPIOPin pin, const bool value,
     const std::chrono::milliseconds & pin_validation_wait_time),
    (override));

  using NiceMock = testing::NiceMock<MockGPIODriver>;
};

class GPIOControllerWrapper : public husarion_ugv_hardware_interfaces::GPIOController
{
public:
  GPIOControllerWrapper(std::shared_ptr<MockGPIODriver> gpio_driver)
  : husarion_ugv_hardware_interfaces::GPIOController(gpio_driver)
  {
  }

  void WatchdogEnable() { this->watchdog_->TurnOn(); }
  void WatchdogDisable() { this->watchdog_->TurnOff(); }
  bool IsWatchdogEnabled() { return this->watchdog_->IsWatchdogEnabled(); }
};

class TestGPIOController : public ::testing::Test
{
public:
  TestGPIOController();
  ~TestGPIOController() { gpio_controller_wrapper_.reset(); }

protected:
  float GetRobotVersion();

  std::shared_ptr<MockGPIODriver::NiceMock> gpio_driver_;
  std::unique_ptr<GPIOControllerWrapper> gpio_controller_wrapper_;
  static constexpr int watchdog_edges_per_100ms_ = 10;
};

TestGPIOController::TestGPIOController()
{
  gpio_driver_ = std::make_shared<MockGPIODriver::NiceMock>();

  // Mock methods called during the initialization process
  ON_CALL(*gpio_driver_, SetPinValue(GPIOPin::VMOT_ON, true, testing::_))
    .WillByDefault(testing::Return(true));
  ON_CALL(*gpio_driver_, SetPinValue(GPIOPin::DRIVER_EN, true, testing::_))
    .WillByDefault(testing::Return(true));
  ON_CALL(*gpio_driver_, IsPinAvailable(GPIOPin::WATCHDOG)).WillByDefault(testing::Return(true));

  gpio_controller_wrapper_ = std::make_unique<GPIOControllerWrapper>(gpio_driver_);
  gpio_controller_wrapper_->Start();
}

struct GPIOTestParam
{
  GPIOPin pin;
  std::function<bool(GPIOControllerWrapper *, bool)> enable_method;
  bool is_inverted = false;
};

class ParametrizedTestGPIOController : public TestGPIOController,
                                       public ::testing::WithParamInterface<GPIOTestParam>
{
};

TEST(TestGPIOControllerInitialization, GPIODriverUninitialized)
{
  std::shared_ptr<MockGPIODriver> gpio_driver;

  EXPECT_THROW(std::make_unique<GPIOControllerWrapper>(gpio_driver), std::runtime_error);
}

TEST(TestGPIOControllerInitialization, WatchdogPinNotAvailable)
{
  auto gpio_driver = std::make_shared<MockGPIODriver>();

  EXPECT_CALL(*gpio_driver, SetPinValue(testing::_, true, testing::_))
    .Times(2)
    .WillRepeatedly(testing::Return(true));
  ON_CALL(*gpio_driver, IsPinAvailable(GPIOPin::WATCHDOG)).WillByDefault(testing::Return(false));

  auto gpio_controller_wrapper = std::make_unique<GPIOControllerWrapper>(gpio_driver);

  EXPECT_THROW(gpio_controller_wrapper->Start(), std::runtime_error);
}

TEST_F(TestGPIOController, TestMotorsInitSuccess)
{
  ON_CALL(*gpio_driver_, IsPinActive(GPIOPin::VMOT_ON)).WillByDefault(testing::Return(true));
  ON_CALL(*gpio_driver_, IsPinActive(GPIOPin::DRIVER_EN)).WillByDefault(testing::Return(true));

  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(GPIOPin::VMOT_ON));
  EXPECT_TRUE(gpio_controller_wrapper_->IsPinActive(GPIOPin::DRIVER_EN));
}

TEST_F(TestGPIOController, TestEStopResetAlreadyDeactivated)
{
  // The E_STOP_RESET pin is already deactivated
  EXPECT_CALL(*gpio_driver_, IsPinActive(GPIOPin::E_STOP_RESET)).WillOnce(testing::Return(true));

  gpio_controller_wrapper_->EStopReset();
}

TEST_P(ParametrizedTestGPIOController, TestGPIOEnableDisable)
{
  const auto & param = GetParam();

  // Set the enable command based on the pin inversion
  auto const enable = param.is_inverted ? false : true;
  auto const disable = !enable;

  // Enable GPIO
  EXPECT_CALL(*gpio_driver_, SetPinValue(param.pin, enable, testing::_))
    .WillOnce(testing::Return(true));
  EXPECT_CALL(*gpio_driver_, IsPinActive(param.pin)).WillOnce(testing::Return(enable));

  param.enable_method(gpio_controller_wrapper_.get(), true);
  EXPECT_EQ(enable, gpio_controller_wrapper_->IsPinActive(param.pin));

  // Disable GPIO
  EXPECT_CALL(*gpio_driver_, SetPinValue(param.pin, disable, testing::_))
    .WillOnce(testing::Return(true));
  EXPECT_CALL(*gpio_driver_, IsPinActive(param.pin)).WillOnce(testing::Return(disable));

  param.enable_method(gpio_controller_wrapper_.get(), false);
  EXPECT_EQ(disable, gpio_controller_wrapper_->IsPinActive(param.pin));
}

INSTANTIATE_TEST_SUITE_P(
  GPIOTests, ParametrizedTestGPIOController,
  ::testing::Values(
    GPIOTestParam{GPIOPin::DRIVER_EN, &GPIOControllerWrapper::MotorPowerEnable},
    GPIOTestParam{GPIOPin::AUX_PW_EN, &GPIOControllerWrapper::AUXPowerEnable},
    GPIOTestParam{GPIOPin::FAN_SW, &GPIOControllerWrapper::FanEnable},
    GPIOTestParam{GPIOPin::VDIG_OFF, &GPIOControllerWrapper::DigitalPowerEnable, true},
    GPIOTestParam{GPIOPin::CHRG_DISABLE, &GPIOControllerWrapper::ChargerEnable, true},
    GPIOTestParam{GPIOPin::LED_SBC_SEL, &GPIOControllerWrapper::LEDControlEnable}));

TEST_F(TestGPIOController, TestQueryControlInterfaceIOStates)
{
  EXPECT_CALL(*gpio_driver_, IsPinActive(testing::_))
    .Times(7)
    .WillRepeatedly(testing::Return(true));

  std::unordered_map<GPIOPin, bool> io_states =
    gpio_controller_wrapper_->QueryControlInterfaceIOStates();

  ASSERT_EQ(io_states.size(), 7);

  for (const auto & [pin, state] : io_states) {
    EXPECT_TRUE(state);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
