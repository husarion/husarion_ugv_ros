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

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_data_converters.hpp"

#include "utils/test_constants.hpp"

void TestFaultFlagMsg(
  const husarion_ugv_msgs::msg::FaultFlag & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 8) {
    throw std::runtime_error("Wrong size of expected_values in TestFaultFlagMsg");
  }

  EXPECT_EQ(msg.overheat, expected_values[0]);
  EXPECT_EQ(msg.overvoltage, expected_values[1]);
  EXPECT_EQ(msg.undervoltage, expected_values[2]);
  EXPECT_EQ(msg.short_circuit, expected_values[3]);
  EXPECT_EQ(msg.emergency_stop, expected_values[4]);
  EXPECT_EQ(msg.motor_or_sensor_setup_fault, expected_values[5]);
  EXPECT_EQ(msg.mosfet_failure, expected_values[6]);
  EXPECT_EQ(msg.default_config_loaded_at_startup, expected_values[7]);
}

void TestScriptFlagMsg(
  const husarion_ugv_msgs::msg::ScriptFlag & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 3) {
    throw std::runtime_error("Wrong size of expected_values in TestScriptFlagMsg");
  }

  EXPECT_EQ(msg.loop_error, expected_values[0]);
  EXPECT_EQ(msg.encoder_disconnected, expected_values[1]);
  EXPECT_EQ(msg.amp_limiter, expected_values[2]);
}

void TestRuntimeErrorMsg(
  const husarion_ugv_msgs::msg::RuntimeError & msg, const std::vector<bool> & expected_values)
{
  if (expected_values.size() != 7) {
    throw std::runtime_error("Wrong size of expected_values in TestFaultFlagMsg");
  }

  EXPECT_EQ(msg.amps_limit_active, expected_values[0]);
  EXPECT_EQ(msg.motor_stall, expected_values[1]);
  EXPECT_EQ(msg.loop_error, expected_values[2]);
  EXPECT_EQ(msg.safety_stop_active, expected_values[3]);
  EXPECT_EQ(msg.forward_limit_triggered, expected_values[4]);
  EXPECT_EQ(msg.reverse_limit_triggered, expected_values[5]);
  EXPECT_EQ(msg.amps_trigger_activated, expected_values[6]);
}

TEST(TestRoboteqDataConverters, CommandConverter)
{
  husarion_ugv_hardware_interfaces::RoboteqVelocityCommandConverter cmd_converter(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);

  const float conversion_factor = 79.789678;

  EXPECT_EQ(cmd_converter.Convert(100.0), 1000);
  EXPECT_EQ(cmd_converter.Convert(10.0), static_cast<std::int32_t>(10.0 * conversion_factor));
  EXPECT_EQ(cmd_converter.Convert(0.0), 0);
  EXPECT_EQ(cmd_converter.Convert(-10.0), static_cast<std::int32_t>(-10.0 * conversion_factor));
  EXPECT_EQ(cmd_converter.Convert(-100.0), -1000);
}

TEST(TestRoboteqDataConverters, MotorState)
{
  using husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  using husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  husarion_ugv_hardware_interfaces::MotorState motor_state(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);

  EXPECT_FLOAT_EQ(motor_state.GetPosition(), 0.0);
  EXPECT_FLOAT_EQ(motor_state.GetVelocity(), 0.0);
  EXPECT_FLOAT_EQ(motor_state.GetTorque(), 0.0);

  husarion_ugv_hardware_interfaces::MotorDriverState feedback1;
  feedback1.pos = 48128;
  feedback1.vel = 1000;
  feedback1.current = 1;
  motor_state.SetData(feedback1);

  EXPECT_FLOAT_EQ(motor_state.GetPosition(), feedback1.pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(motor_state.GetVelocity(), feedback1.vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(motor_state.GetTorque(), feedback1.current * kRbtqCurrentFbToNewtonMeters);

  husarion_ugv_hardware_interfaces::MotorDriverState feedback2;
  feedback2.pos = -48128;
  feedback2.vel = -1000;
  feedback2.current = -1;
  motor_state.SetData(feedback2);

  EXPECT_FLOAT_EQ(motor_state.GetPosition(), feedback2.pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(motor_state.GetVelocity(), feedback2.vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(motor_state.GetTorque(), feedback2.current * kRbtqCurrentFbToNewtonMeters);
}

TEST(TestRoboteqDataConverters, FlagError)
{
  husarion_ugv_hardware_interfaces::FlagError flag_error(
    {"error1", "error2", "error3", "error4", "error5", "error6", "error7", "error8"},
    {"error2", "error6"});

  ASSERT_FALSE(flag_error.IsError());
  EXPECT_EQ(flag_error.GetErrorLog(), "");

  flag_error.SetData(0b00000001);
  ASSERT_TRUE(flag_error.IsError());
  EXPECT_EQ(flag_error.GetErrorLog(), "error1 ");

  flag_error.SetData(0b00100010);
  ASSERT_FALSE(flag_error.IsError());
  EXPECT_EQ(flag_error.GetErrorLog(), "");

  flag_error.SetData(0b10000001);
  ASSERT_TRUE(flag_error.IsError());
  EXPECT_EQ(flag_error.GetErrorLog(), "error1 error8 ");
}

TEST(TestRoboteqDataConverters, FaultFlag)
{
  husarion_ugv_hardware_interfaces::FaultFlag fault_flag;

  fault_flag.SetData(0b00000001);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {true, false, false, false, false, false, false, false});
  fault_flag.SetData(0b00000010);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, true, false, false, false, false, false, false});
  fault_flag.SetData(0b00000100);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, true, false, false, false, false, false});
  fault_flag.SetData(0b00001000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, true, false, false, false, false});
  fault_flag.SetData(0b00010000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, true, false, false, false});
  fault_flag.SetData(0b00100000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, true, false, false});
  fault_flag.SetData(0b01000000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, true, false});
  fault_flag.SetData(0b10000000);
  TestFaultFlagMsg(
    fault_flag.GetMessage(), {false, false, false, false, false, false, false, true});

  fault_flag.SetData(0b00100010);
  TestFaultFlagMsg(fault_flag.GetMessage(), {false, true, false, false, false, true, false, false});
}

TEST(TestRoboteqDataConverters, ScriptFlag)
{
  husarion_ugv_hardware_interfaces::ScriptFlag script_flag;

  script_flag.SetData(0b00000001);
  TestScriptFlagMsg(script_flag.GetMessage(), {true, false, false});
  script_flag.SetData(0b00000010);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, true, false});
  script_flag.SetData(0b00000100);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, false, true});

  script_flag.SetData(0b00100110);
  TestScriptFlagMsg(script_flag.GetMessage(), {false, true, true});
}

TEST(TestRoboteqDataConverters, RuntimeError)
{
  husarion_ugv_hardware_interfaces::RuntimeError runtime_error;

  runtime_error.SetData(0b00000001);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {true, false, false, false, false, false, false});
  runtime_error.SetData(0b00000010);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, true, false, false, false, false, false});
  runtime_error.SetData(0b00000100);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, true, false, false, false, false});
  runtime_error.SetData(0b00001000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, true, false, false, false});
  runtime_error.SetData(0b00010000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, true, false, false});
  runtime_error.SetData(0b00100000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, false, true, false});
  runtime_error.SetData(0b01000000);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, false, false, false, true});

  runtime_error.SetData(0b00100010);
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, true, false, false, false, true, false});

  runtime_error.SetData(0b00001000);
  // fault flag still should be set, it just won't be treated as an error
  TestRuntimeErrorMsg(runtime_error.GetMessage(), {false, false, false, true, false, false, false});
}

TEST(TestRoboteqDataConverters, DriverState)
{
  husarion_ugv_hardware_interfaces::RoboteqDriverState driver_state;

  EXPECT_FLOAT_EQ(driver_state.GetTemperature(), 0.0);
  EXPECT_FLOAT_EQ(driver_state.GetVoltage(), 0.0);
  EXPECT_FLOAT_EQ(driver_state.GetCurrent(), 0.0);

  driver_state.SetTemperature(32);
  driver_state.SetVoltage(365);
  driver_state.SetBatteryCurrent1(15);
  driver_state.SetBatteryCurrent2(20);

  EXPECT_FLOAT_EQ(driver_state.GetTemperature(), 32);
  EXPECT_FLOAT_EQ(driver_state.GetVoltage(), 36.5);
  EXPECT_FLOAT_EQ(driver_state.GetCurrent(), 3.5);
}

TEST(TestRoboteqDataConverters, DriverData)
{
  using husarion_ugv_hardware_interfaces::RoboteqDriver;
  using husarion_ugv_hardware_interfaces_test::kRbtqCurrentFbToNewtonMeters;
  using husarion_ugv_hardware_interfaces_test::kRbtqPosFbToRad;
  using husarion_ugv_hardware_interfaces_test::kRbtqVelFbToRadPerSec;

  husarion_ugv_hardware_interfaces::DriverData roboteq_data(
    husarion_ugv_hardware_interfaces_test::kDrivetrainSettings);

  ASSERT_FALSE(roboteq_data.IsError());

  const husarion_ugv_hardware_interfaces::MotorDriverState channel_1_state = {
    0, 0, 0, {0, 0}, {0, 0}};
  const husarion_ugv_hardware_interfaces::MotorDriverState channel_2_state = {
    48128, 1000, 1, {0, 0}, {0, 0}};

  roboteq_data.SetMotorsStates(channel_1_state, channel_2_state, true);

  ASSERT_TRUE(roboteq_data.IsError());
  ASSERT_TRUE(roboteq_data.IsMotorStatesDataTimedOut());

  roboteq_data.SetMotorsStates(channel_1_state, channel_2_state, false);

  ASSERT_FALSE(roboteq_data.IsError());

  roboteq_data.SetCANError(true);

  ASSERT_TRUE(roboteq_data.IsError());
  ASSERT_TRUE(roboteq_data.IsCANError());

  roboteq_data.SetCANError(false);
  ASSERT_FALSE(roboteq_data.IsError());

  EXPECT_FLOAT_EQ(roboteq_data.GetMotorState(RoboteqDriver::kChannel1).GetPosition(), 0.0);
  EXPECT_FLOAT_EQ(roboteq_data.GetMotorState(RoboteqDriver::kChannel1).GetVelocity(), 0.0);
  EXPECT_FLOAT_EQ(roboteq_data.GetMotorState(RoboteqDriver::kChannel1).GetTorque(), 0.0);
  EXPECT_FLOAT_EQ(
    roboteq_data.GetMotorState(RoboteqDriver::kChannel2).GetPosition(),
    channel_2_state.pos * kRbtqPosFbToRad);
  EXPECT_FLOAT_EQ(
    roboteq_data.GetMotorState(RoboteqDriver::kChannel2).GetVelocity(),
    channel_2_state.vel * kRbtqVelFbToRadPerSec);
  EXPECT_FLOAT_EQ(
    roboteq_data.GetMotorState(RoboteqDriver::kChannel2).GetTorque(),
    channel_2_state.current * kRbtqCurrentFbToNewtonMeters);

  husarion_ugv_hardware_interfaces::DriverState state;

  state.fault_flags = 0b00000001;
  state.script_flags = 0b00000010;
  state.runtime_stat_flag_channel_1 = 0b00010000;
  state.runtime_stat_flag_channel_2 = 0b00000100;

  state.mcu_temp = 32;
  state.heatsink_temp = 31;
  state.battery_voltage = 365;
  state.battery_current_1 = 15;
  state.battery_current_2 = 20;

  roboteq_data.SetDriverState(state, false);

  ASSERT_TRUE(roboteq_data.IsError());

  EXPECT_TRUE(roboteq_data.GetFaultFlag().GetMessage().overheat);
  EXPECT_TRUE(roboteq_data.GetScriptFlag().GetMessage().encoder_disconnected);
  EXPECT_TRUE(roboteq_data.GetRuntimeError(RoboteqDriver::kChannel2).GetMessage().loop_error);
  EXPECT_TRUE(
    roboteq_data.GetRuntimeError(RoboteqDriver::kChannel1).GetMessage().forward_limit_triggered);

  EXPECT_FLOAT_EQ(roboteq_data.GetDriverState().GetTemperature(), 32);
  EXPECT_FLOAT_EQ(roboteq_data.GetDriverState().GetHeatsinkTemperature(), 31);
  EXPECT_FLOAT_EQ(roboteq_data.GetDriverState().GetVoltage(), 36.5);
  EXPECT_FLOAT_EQ(roboteq_data.GetDriverState().GetCurrent(), 3.5);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
