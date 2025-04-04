// Copyright 2023 Husarion sp. z o.o.
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

#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/driver.hpp"
#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_driver.hpp"

#include "utils/fake_can_socket.hpp"
#include "utils/mock_roboteq.hpp"
#include "utils/test_constants.hpp"

#include "husarion_ugv_utils/test/test_utils.hpp"

class TestRoboteqDriver : public ::testing::Test
{
public:
  TestRoboteqDriver();

  ~TestRoboteqDriver();

  void BootRoboteqDriver();

protected:
  bool TestBoot();

  std::unique_ptr<husarion_ugv_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<husarion_ugv_hardware_interfaces_test::MockRoboteq> mock_roboteq_;
  std::unique_ptr<husarion_ugv_hardware_interfaces::CANopenManager> canopen_manager_;
  std::shared_ptr<husarion_ugv_hardware_interfaces::RoboteqDriver> roboteq_driver_;
};

TestRoboteqDriver::TestRoboteqDriver()
{
  can_socket_ = std::make_unique<husarion_ugv_hardware_interfaces_test::FakeCANSocket>(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.can_interface_name);
  can_socket_->Initialize();

  canopen_manager_ = std::make_unique<husarion_ugv_hardware_interfaces::CANopenManager>(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings);

  mock_roboteq_ = std::make_unique<husarion_ugv_hardware_interfaces_test::MockRoboteq>();
  mock_roboteq_->Start(std::chrono::milliseconds(10), std::chrono::milliseconds(50));
  canopen_manager_->Initialize();

  roboteq_driver_ = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqDriver>(
    canopen_manager_->GetMaster(), 1, std::chrono::milliseconds(100));

  canopen_manager_->Activate();
}

TestRoboteqDriver::~TestRoboteqDriver()
{
  roboteq_driver_.reset();
  canopen_manager_->Deinitialize();
  mock_roboteq_->Stop();
  mock_roboteq_.reset();
  can_socket_->Deinitialize();
}

void TestRoboteqDriver::BootRoboteqDriver()
{
  auto motor_1 = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqMotorDriver>(
    roboteq_driver_, husarion_ugv_hardware_interfaces::RoboteqDriver::kChannel1);
  auto motor_2 = std::make_shared<husarion_ugv_hardware_interfaces::RoboteqMotorDriver>(
    roboteq_driver_, husarion_ugv_hardware_interfaces::RoboteqDriver::kChannel2);

  roboteq_driver_->AddMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT, motor_1);
  roboteq_driver_->AddMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT, motor_2);
  auto future = roboteq_driver_->Boot();
  future.wait();
}

bool TestRoboteqDriver::TestBoot()
{
  auto future = roboteq_driver_->Boot();
  const auto future_status = future.wait_for(std::chrono::milliseconds(500));

  if (future_status != std::future_status::ready) {
    return false;
  }

  try {
    future.get();
  } catch (const std::exception & e) {
    return false;
  }

  return true;
}

TEST_F(TestRoboteqDriver, BootRoboteqDriver) { ASSERT_TRUE(TestBoot()); }

TEST_F(TestRoboteqDriver, BootErrorDeviceType)
{
  const auto device_type_id = 0x1000;
  const auto device_type_subid = 0;
  mock_roboteq_->GetDriver()->SetOnReadWait<std::uint32_t>(
    device_type_id, device_type_subid, 100000);
  EXPECT_FALSE(TestBoot());
}

TEST_F(TestRoboteqDriver, BootErrorVendorId)
{
  const auto vendor_id_id = 0x1018;
  const auto vendor_id_subid = 1;
  mock_roboteq_->GetDriver()->SetOnReadWait<std::uint32_t>(vendor_id_id, vendor_id_subid, 100000);
  EXPECT_FALSE(TestBoot());
}

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStates)
{
  using husarion_ugv_hardware_interfaces_test::DriverChannel;

  const std::int32_t motor_1_pos = 101;
  const std::int32_t motor_1_vel = 102;
  const std::int32_t motor_1_current = 103;
  const std::int32_t motor_2_pos = 201;
  const std::int32_t motor_2_vel = 202;
  const std::int32_t motor_2_current = 203;

  BootRoboteqDriver();

  mock_roboteq_->GetDriver()->SetPosition(DriverChannel::CHANNEL1, motor_1_pos);
  mock_roboteq_->GetDriver()->SetPosition(DriverChannel::CHANNEL2, motor_2_pos);

  mock_roboteq_->GetDriver()->SetVelocity(DriverChannel::CHANNEL1, motor_1_vel);
  mock_roboteq_->GetDriver()->SetVelocity(DriverChannel::CHANNEL2, motor_2_vel);

  mock_roboteq_->GetDriver()->SetCurrent(DriverChannel::CHANNEL1, motor_1_current);
  mock_roboteq_->GetDriver()->SetCurrent(DriverChannel::CHANNEL2, motor_2_current);

  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_1 =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
      ->ReadState();
  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_2 =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
      ->ReadState();

  EXPECT_EQ(motor_driver_state_1.pos, motor_1_pos);
  EXPECT_EQ(motor_driver_state_1.vel, motor_1_vel);
  EXPECT_EQ(motor_driver_state_1.current, motor_1_current);

  EXPECT_EQ(motor_driver_state_2.pos, motor_2_pos);
  EXPECT_EQ(motor_driver_state_2.vel, motor_2_vel);
  EXPECT_EQ(motor_driver_state_2.current, motor_2_current);
}

TEST_F(TestRoboteqDriver, ReadRoboteqMotorStatesTimestamps)
{
  BootRoboteqDriver();

  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_1 =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
      ->ReadState();

  // based on publishing frequency in the Roboteq mock (100Hz)
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_2 =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
      ->ReadState();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(motor_driver_state_2.pos_timestamp) -
      lely::util::from_timespec(motor_driver_state_1.pos_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(motor_driver_state_2.pos_timestamp) -
      lely::util::from_timespec(motor_driver_state_1.pos_timestamp),
    std::chrono::milliseconds(5));

  EXPECT_LE(
    lely::util::from_timespec(motor_driver_state_2.vel_current_timestamp) -
      lely::util::from_timespec(motor_driver_state_1.vel_current_timestamp),
    std::chrono::milliseconds(15));

  EXPECT_GE(
    lely::util::from_timespec(motor_driver_state_2.vel_current_timestamp) -
      lely::util::from_timespec(motor_driver_state_1.vel_current_timestamp),
    std::chrono::milliseconds(5));
}

TEST_F(TestRoboteqDriver, ReadState)
{
  using husarion_ugv_hardware_interfaces_test::DriverChannel;
  using husarion_ugv_hardware_interfaces_test::DriverFaultFlags;
  using husarion_ugv_hardware_interfaces_test::DriverRuntimeErrors;
  using husarion_ugv_hardware_interfaces_test::DriverScriptFlags;

  const std::int16_t temp = 30;
  const std::int16_t heatsink_temp = 31;
  const std::uint16_t volt = 400;
  const std::int16_t battery_current_1 = 10;
  const std::int16_t battery_current_2 = 30;

  mock_roboteq_->GetDriver()->SetTemperature(temp);
  mock_roboteq_->GetDriver()->SetHeatsinkTemperature(heatsink_temp);
  mock_roboteq_->GetDriver()->SetVoltage(volt);
  mock_roboteq_->GetDriver()->SetBatteryCurrent1(battery_current_1);
  mock_roboteq_->GetDriver()->SetBatteryCurrent2(battery_current_2);

  mock_roboteq_->GetDriver()->SetDriverFaultFlag(DriverFaultFlags::OVERHEAT);
  mock_roboteq_->GetDriver()->SetDriverScriptFlag(DriverScriptFlags::ENCODER_DISCONNECTED);
  mock_roboteq_->GetDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL1, DriverRuntimeErrors::LOOP_ERROR);
  mock_roboteq_->GetDriver()->SetDriverRuntimeError(
    DriverChannel::CHANNEL2, DriverRuntimeErrors::SAFETY_STOP_ACTIVE);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  husarion_ugv_hardware_interfaces::DriverState driver_state = roboteq_driver_->ReadState();

  EXPECT_EQ(driver_state.mcu_temp, temp);
  EXPECT_EQ(driver_state.heatsink_temp, heatsink_temp);
  EXPECT_EQ(driver_state.battery_voltage, volt);
  EXPECT_EQ(driver_state.battery_current_1, battery_current_1);
  EXPECT_EQ(driver_state.battery_current_2, battery_current_2);

  EXPECT_EQ(driver_state.fault_flags, 0b00000001);
  EXPECT_EQ(driver_state.script_flags, 0b00000010);
  EXPECT_EQ(driver_state.runtime_stat_flag_channel_1, 0b00000100);
  EXPECT_EQ(driver_state.runtime_stat_flag_channel_2, 0b00001000);
}

TEST_F(TestRoboteqDriver, ReadStateTimestamp)
{
  BootRoboteqDriver();

  husarion_ugv_hardware_interfaces::DriverState driver_state_1 = roboteq_driver_->ReadState();

  // based on publishing frequency in the Roboteq mock (20Hz)
  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  husarion_ugv_hardware_interfaces::DriverState driver_state_2 = roboteq_driver_->ReadState();

  // feedback is published with a 100ms period, to check if timestamps are accurate, it is checked
  // if consecutive messages will have timestamps 100ms + some threshold apart
  EXPECT_LE(
    lely::util::from_timespec(driver_state_2.flags_current_timestamp) -
      lely::util::from_timespec(driver_state_1.flags_current_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(driver_state_2.flags_current_timestamp) -
      lely::util::from_timespec(driver_state_1.flags_current_timestamp),
    std::chrono::milliseconds(25));

  EXPECT_LE(
    lely::util::from_timespec(driver_state_2.voltages_temps_timestamp) -
      lely::util::from_timespec(driver_state_1.voltages_temps_timestamp),
    std::chrono::milliseconds(75));

  EXPECT_GE(
    lely::util::from_timespec(driver_state_2.voltages_temps_timestamp) -
      lely::util::from_timespec(driver_state_1.voltages_temps_timestamp),
    std::chrono::milliseconds(25));
}

TEST_F(TestRoboteqDriver, SendRoboteqCmd)
{
  using husarion_ugv_hardware_interfaces_test::DriverChannel;

  const std::int32_t motor_1_v = 10;
  const std::int32_t motor_2_v = 20;

  BootRoboteqDriver();

  roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
    ->SendCmdVel(motor_1_v);
  roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
    ->SendCmdVel(motor_2_v);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), motor_1_v);
  EXPECT_EQ(mock_roboteq_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL2), motor_2_v);
}

TEST_F(TestRoboteqDriver, ResetRoboteqScript)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetResetRoboteqScript(65);
  roboteq_driver_->ResetScript();

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetResetRoboteqScript(), 2);
}

TEST_F(TestRoboteqDriver, ResetRoboteqScriptSDOTimeoutReset)
{
  BootRoboteqDriver();

  mock_roboteq_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x2018, 0, 100000);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->ResetScript(); }, "SDO protocol timed out"));

  mock_roboteq_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x2018, 0, 0);
  EXPECT_NO_THROW(roboteq_driver_->ResetScript());
}

TEST_F(TestRoboteqDriver, RoboteqTurnOnEStop)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetTurnOnEStop(65);
  roboteq_driver_->TurnOnEStop();

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetTurnOnEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnEStopTimeout)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x200C, 0, 100000);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOnEStop(); }, "SDO protocol timed out"));
}

TEST_F(TestRoboteqDriver, TurnOffEStop)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetTurnOffEStop(65);
  roboteq_driver_->TurnOffEStop();

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetTurnOffEStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOffEStopTimeout)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x200D, 0, 100000);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOffEStop(); }, "SDO protocol timed out"));
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel1)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetTurnOnSafetyStop(67);
  roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
    ->TurnOnSafetyStop();

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetTurnOnSafetyStop(), 1);
}

TEST_F(TestRoboteqDriver, TurnOnSafetyStopChannel2)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetTurnOnSafetyStop(65);
  roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
    ->TurnOnSafetyStop();

  EXPECT_EQ(mock_roboteq_->GetDriver()->GetTurnOnSafetyStop(), 2);
}

TEST_F(TestRoboteqDriver, WriteTimeout)
{
  BootRoboteqDriver();
  mock_roboteq_->GetDriver()->SetOnWriteWait<std::uint8_t>(0x202C, 0, 200000);
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() {
      roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
        ->TurnOnSafetyStop();
    },
    "SDO protocol timed out"));
}

// OnCanError/OnHeartbeatTimeout isn't tested, because it reacts to lower-level CAN errors (CRC),
// which are hard to simulate, but it would be nice to add it

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();
  return result;
}
