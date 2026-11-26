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

#include <stdexcept>

#include <gtest/gtest.h>

#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/cmd_write_status.hpp>
#include <husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp>

TEST(TestRoboteqErrorFilter, InitialState)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, FilterError)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, Error)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, FilterSecondError)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, false);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, SecondError)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, ErrorSingle)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, ClearErrors)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_DRIVER_STATE, true);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);

  ASSERT_TRUE(roboteq_error_filter.IsError());
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.SetClearErrorsFlag();
  // Has to trigger at least one update to clear errors
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, ClearErrorsCounters)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.SetClearErrorsFlag();

  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::READ_PDO_MOTOR_STATES, true);

  ASSERT_FALSE(roboteq_error_filter.IsError());
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_MOTOR_STATES));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::READ_PDO_DRIVER_STATE));
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestCmdWriteStatus, FailureStaysUntilAWriteSucceeds)
{
  husarion_ugv_hardware_interfaces::CmdWriteStatus status;
  EXPECT_NO_THROW(status.ThrowIfFailed(0));

  status.RecordFailure(0, "channel 1: write failed");
  EXPECT_THROW(status.ThrowIfFailed(0), std::runtime_error);
  // No write completed since, so the failure is still the newest result.
  EXPECT_THROW(status.ThrowIfFailed(0), std::runtime_error);

  status.RecordSuccess(0);
  EXPECT_NO_THROW(status.ThrowIfFailed(0));
}

TEST(TestCmdWriteStatus, ChannelsAreIndependent)
{
  husarion_ugv_hardware_interfaces::CmdWriteStatus status;
  status.RecordFailure(1, "channel 2: write failed");

  EXPECT_NO_THROW(status.ThrowIfFailed(0));
  try {
    status.ThrowIfFailed(1);
    FAIL() << "The failure of channel 2 was not reported.";
  } catch (const std::runtime_error & e) {
    EXPECT_STREQ(e.what(), "Velocity command write failed, channel 2: write failed");
  }
}

TEST(TestCmdWriteStatus, LaggingFailedWritesReachTheFilter)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  // Count every control cycle the way UGVSystem::HandleRobotDriverWriteOperation does.
  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(3, 2, 1, 1);
  husarion_ugv_hardware_interfaces::CmdWriteStatus status;
  const auto control_cycle = [&]() {
    try {
      status.ThrowIfFailed(0);
      roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, false);
    } catch (const std::runtime_error &) {
      roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true);
    }
  };

  // The master runs the command task only every other cycle, and every write fails. A cycle
  // whose task has not run yet must not reset the consecutive error count.
  for (int cycle = 0; cycle < 4; ++cycle) {
    if (cycle % 2 == 0) {
      status.RecordFailure(0, "channel 1: write failed");
    }
    control_cycle();
  }

  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::WRITE_PDO_CMDS));
}

TEST(TestRoboteqErrorFilter, GetErrorDescriptionMap)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true);

  const auto error_map = roboteq_error_filter.GetErrorDescriptionMap();
  EXPECT_EQ(4, error_map.size());
  EXPECT_TRUE(error_map.at("Motor controller fault"));
  EXPECT_FALSE(error_map.at("Motor state data timing out"));
}

TEST(TestRoboteqErrorFilter, UnexcusedError)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);

  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true, true);
  EXPECT_TRUE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
  EXPECT_FALSE(roboteq_error_filter.IsUnexcusedError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  // A real fault keeps the latched error unexcused after it went away.
  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true, false);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, true, true);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
  EXPECT_TRUE(roboteq_error_filter.IsUnexcusedError(ErrorsFilterIds::ROBOTEQ_DRIVER));

  roboteq_error_filter.SetClearErrorsFlag();
  roboteq_error_filter.UpdateError(ErrorsFilterIds::ROBOTEQ_DRIVER, false);
  EXPECT_FALSE(roboteq_error_filter.IsError(ErrorsFilterIds::ROBOTEQ_DRIVER));
  EXPECT_FALSE(roboteq_error_filter.IsUnexcusedError(ErrorsFilterIds::ROBOTEQ_DRIVER));
}

TEST(TestRoboteqErrorFilter, UnexcusedErrorInARunThatLatches)
{
  using husarion_ugv_hardware_interfaces::ErrorsFilterIds;

  // The excused error of the second sample latches it, but the run had a real fault in it.
  husarion_ugv_hardware_interfaces::RoboteqErrorFilter roboteq_error_filter(2, 2, 1, 1);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true, false);
  roboteq_error_filter.UpdateError(ErrorsFilterIds::WRITE_PDO_CMDS, true, true);
  EXPECT_TRUE(roboteq_error_filter.IsUnexcusedError(ErrorsFilterIds::WRITE_PDO_CMDS));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
