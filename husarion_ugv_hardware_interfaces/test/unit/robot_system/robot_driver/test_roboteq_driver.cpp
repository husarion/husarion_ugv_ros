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

#include <pthread.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
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

  // Blocks an executor until Release() is called or the guard is destroyed, so a failed ASSERT
  // does not leave the fixture teardown waiting on a blocked executor.
  class ExecutorBlock
  {
  public:
    explicit ExecutorBlock(lely::ev::Executor executor);
    ~ExecutorBlock() { Release(); }
    void Release();

  private:
    std::promise<void> release_;
    bool released_ = false;
  };

  std::unique_ptr<ExecutorBlock> BlockMasterExecutor();
  std::unique_ptr<ExecutorBlock> BlockDriverExecutor();

  // Returns once every task posted to the master's executor before this call has run.
  bool WaitForMasterExecutor();

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
    canopen_manager_->GetMaster(), 1, std::chrono::milliseconds(100), 65);

  canopen_manager_->Activate();
}

TestRoboteqDriver::~TestRoboteqDriver()
{
  // The same order as RoboteqRobotDriver::Deinitialize(): no posted task runs after the driver
  // is gone.
  canopen_manager_->StopCommunication();
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

TestRoboteqDriver::ExecutorBlock::ExecutorBlock(lely::ev::Executor executor)
{
  auto released = release_.get_future().share();
  auto blocked = std::make_shared<std::promise<void>>();
  auto blocked_future = blocked->get_future();
  executor.post([released, blocked]() {
    blocked->set_value();
    released.wait();
  });
  blocked_future.wait();
}

void TestRoboteqDriver::ExecutorBlock::Release()
{
  if (!released_) {
    released_ = true;
    release_.set_value();
  }
}

std::unique_ptr<TestRoboteqDriver::ExecutorBlock> TestRoboteqDriver::BlockMasterExecutor()
{
  return std::make_unique<ExecutorBlock>(canopen_manager_->GetMaster()->GetExecutor());
}

std::unique_ptr<TestRoboteqDriver::ExecutorBlock> TestRoboteqDriver::BlockDriverExecutor()
{
  return std::make_unique<ExecutorBlock>(roboteq_driver_->GetExecutor());
}

bool TestRoboteqDriver::WaitForMasterExecutor()
{
  auto done = std::make_shared<std::promise<void>>();
  auto done_future = done->get_future();
  canopen_manager_->GetMaster()->GetExecutor().post([done]() { done->set_value(); });
  return done_future.wait_for(std::chrono::seconds(1)) == std::future_status::ready;
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

TEST_F(TestRoboteqDriver, ThreadsAreNamed)
{
  // The name is set by the first task on each executor, before any lely callback runs there.
  const auto thread_name = [](lely::ev::Executor executor) {
    auto name = std::make_shared<std::promise<std::string>>();
    auto name_future = name->get_future();
    executor.post([name]() {
      char buffer[16] = {};
      pthread_getname_np(pthread_self(), buffer, sizeof(buffer));
      name->set_value(buffer);
    });
    if (name_future.wait_for(std::chrono::seconds(1)) != std::future_status::ready) {
      return std::string("<timeout>");
    }
    return name_future.get();
  };

  EXPECT_EQ(thread_name(roboteq_driver_->GetExecutor()), "rpdo-dispatch");
  EXPECT_EQ(thread_name(canopen_manager_->GetMaster()->GetExecutor()), "lely-io");
}

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

  // The values are mirrored one mapped object at a time as a frame is dispatched, so a single read
  // after one frame period can see part of a frame on a loaded machine. Wait for all of them.
  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_1;
  husarion_ugv_hardware_interfaces::MotorDriverState motor_driver_state_2;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(1);
  do {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    motor_driver_state_1 = roboteq_driver_
                             ->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
                             ->ReadState();
    motor_driver_state_2 = roboteq_driver_
                             ->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
                             ->ReadState();
  } while ((motor_driver_state_1.pos != motor_1_pos || motor_driver_state_1.vel != motor_1_vel ||
            motor_driver_state_1.current != motor_1_current ||
            motor_driver_state_2.pos != motor_2_pos || motor_driver_state_2.vel != motor_2_vel ||
            motor_driver_state_2.current != motor_2_current) &&
           std::chrono::steady_clock::now() < deadline);

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

  // The mock publishes every 10 ms, so two reads 10 ms apart should see timestamps about one period
  // apart. A loaded machine can deliver two frames back to back, so a few samples are taken and one
  // of them has to show the period.
  using std::chrono::milliseconds;
  bool period_seen = false;
  for (int attempt = 0; attempt < 5 && !period_seen; ++attempt) {
    const auto motor_driver_state_1 =
      roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT)
        ->ReadState();
    std::this_thread::sleep_for(milliseconds(10));
    const auto motor_driver_state_2 =
      roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::RIGHT)
        ->ReadState();

    const auto pos_delta = lely::util::from_timespec(motor_driver_state_2.pos_timestamp) -
                           lely::util::from_timespec(motor_driver_state_1.pos_timestamp);
    const auto vel_current_delta =
      lely::util::from_timespec(motor_driver_state_2.vel_current_timestamp) -
      lely::util::from_timespec(motor_driver_state_1.vel_current_timestamp);
    period_seen = pos_delta >= milliseconds(5) && pos_delta <= milliseconds(15) &&
                  vel_current_delta >= milliseconds(5) && vel_current_delta <= milliseconds(15);
  }
  EXPECT_TRUE(period_seen);
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

TEST_F(TestRoboteqDriver, SendRoboteqCmdKeepsNewest)
{
  using husarion_ugv_hardware_interfaces_test::DriverChannel;

  BootRoboteqDriver();

  auto motor_driver =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT);

  // The probe is queued after the first command and before the rest. A task that wrote each
  // command in turn would show the probe 1. The coalesced task writes the newest command.
  // The velocity command object, RoboteqCANObjects::cmd_id in the driver.
  static constexpr std::uint16_t kCmdId = 0x2000;
  auto probe = std::make_shared<std::promise<std::int32_t>>();
  auto probe_future = probe->get_future();
  auto block = BlockMasterExecutor();
  motor_driver->SendCmdVel(1);
  std::weak_ptr<husarion_ugv_hardware_interfaces::RoboteqDriver> weak_driver = roboteq_driver_;
  canopen_manager_->GetMaster()->GetExecutor().post([weak_driver, probe]() {
    if (auto driver = weak_driver.lock()) {
      probe->set_value(static_cast<std::int32_t>(
        driver->tpdo_mapped[kCmdId][husarion_ugv_hardware_interfaces::RoboteqDriver::kChannel1]));
    }
  });
  for (std::int32_t cmd = 2; cmd <= 100; ++cmd) {
    motor_driver->SendCmdVel(cmd);
  }
  block->Release();

  ASSERT_EQ(probe_future.wait_for(std::chrono::seconds(1)), std::future_status::ready);
  EXPECT_EQ(probe_future.get(), 100);

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(mock_roboteq_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 100);
}

TEST_F(TestRoboteqDriver, CommandTaskAfterDriverDestroyedDoesNothing)
{
  using husarion_ugv_hardware_interfaces_test::DriverChannel;

  BootRoboteqDriver();
  auto motor_driver =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT);

  // The command task is queued behind a blocked executor and runs after the driver is gone. The
  // driver goes on this thread while the master runs no task of it, which the weak pointer
  // covers; dropping drivers with the loop stopped only keeps the master's own thread from being
  // the last owner.
  auto block = BlockMasterExecutor();
  motor_driver->SendCmdVel(7);
  std::weak_ptr<husarion_ugv_hardware_interfaces::RoboteqDriver> weak_driver = roboteq_driver_;
  roboteq_driver_.reset();
  ASSERT_TRUE(weak_driver.expired());
  block->Release();

  ASSERT_TRUE(WaitForMasterExecutor());
  EXPECT_NE(mock_roboteq_->GetDriver()->GetRoboteqCmd(DriverChannel::CHANNEL1), 7);
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

TEST_F(TestRoboteqDriver, SDOWriteNotSentWhenMasterStopped)
{
  BootRoboteqDriver();
  canopen_manager_->StopCommunication();

  // Nothing runs the posted write, so the wait ends at the SDO timeout plus the margin.
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOnEStop(); }, "SDO write not sent"));
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(3));
}

TEST_F(TestRoboteqDriver, AbandonedSDOWriteIsNeverSent)
{
  BootRoboteqDriver();
  // Count the writes, not the value: the blocked executor misses heartbeats, and the master then
  // resets the slave, which clears the value.
  const auto writes = mock_roboteq_->GetDriver()->CountWrites<std::uint8_t>(0x200D, 0);
  roboteq_driver_->TurnOffEStop();
  ASSERT_EQ(writes->load(), 1);

  // The write waits behind a blocked executor past its deadline, then the executor runs again.
  // A failed e_stop_reset must not release the motor controllers later.
  auto block = BlockMasterExecutor();
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOffEStop(); }, "SDO write not sent"));
  block->Release();

  // The dropped write was queued before this sentinel, so its turn has passed once it runs.
  ASSERT_TRUE(WaitForMasterExecutor());
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  EXPECT_EQ(writes->load(), 1);
}

TEST_F(TestRoboteqDriver, StalledMasterNeverSendsTheReleaseLate)
{
  BootRoboteqDriver();
  // Count the writes, not the values: the counts survive the slave reset that can follow the stall.
  const auto releases = mock_roboteq_->GetDriver()->CountWrites<std::uint8_t>(0x200D, 0);
  roboteq_driver_->TurnOffEStop();
  ASSERT_EQ(releases->load(), 1);

  // A release and the latch that should undo it both wait behind a stalled master. The release
  // is dropped. The latch stays queued and reports that, whether it then gets through is up to
  // the master: the stall outlasts the consumer heartbeat time.
  auto block = BlockMasterExecutor();
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOffEStop(); }, "SDO write not sent"));
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOnEStop(); }, "it may still be sent"));
  block->Release();

  // Both queued tasks have had their turn once the sentinel runs.
  ASSERT_TRUE(WaitForMasterExecutor());
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  EXPECT_EQ(releases->load(), 1);
}

TEST_F(TestRoboteqDriver, WedgedMasterFailsVelocityCommands)
{
  BootRoboteqDriver();
  auto motor_driver =
    roboteq_driver_->GetMotorDriver(husarion_ugv_hardware_interfaces::MotorNames::LEFT);

  auto block = BlockMasterExecutor();
  EXPECT_NO_THROW(motor_driver->SendCmdVel(1));
  std::this_thread::sleep_for(std::chrono::milliseconds(150));
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { motor_driver->SendCmdVel(2); }, "the CANopen master does not run it"));
  block->Release();

  ASSERT_TRUE(WaitForMasterExecutor());
  EXPECT_NO_THROW(motor_driver->SendCmdVel(3));
}

TEST_F(TestRoboteqDriver, UnconfirmedSDOWriteMayHaveReachedTheController)
{
  BootRoboteqDriver();
  const auto writes = mock_roboteq_->GetDriver()->CountWrites<std::uint8_t>(0x200D, 0);

  // The confirmation is delivered on the driver's own executor. With that one blocked, the write
  // goes out but the caller never learns how it ended.
  auto block = BlockDriverExecutor();
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { roboteq_driver_->TurnOffEStop(); }, "may have reached the controller"));
  EXPECT_EQ(writes->load(), 1);
  block->Release();
}

// OnCanError/OnHeartbeatTimeout isn't tested, because it reacts to lower-level CAN errors (CRC),
// which are hard to simulate, but it would be nice to add it

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();
  return result;
}
