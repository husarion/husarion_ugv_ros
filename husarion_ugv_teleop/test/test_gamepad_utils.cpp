// Copyright 2026 Husarion sp. z o.o.
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

#include <unistd.h>

#include <filesystem>
#include <string>
#include <system_error>

#include <gtest/gtest.h>

#include "husarion_ugv_teleop/gamepad_utils.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using husarion_ugv_teleop::DiagnoseGamepadLevel;
using husarion_ugv_teleop::JoystickPresent;

TEST(TestDiagnoseGamepadLevel, NoGamepadIsOK)
{
  EXPECT_EQ(DiagnosticStatus::OK, DiagnoseGamepadLevel(false, false));
}

TEST(TestDiagnoseGamepadLevel, ConnectedGamepadIsOK)
{
  EXPECT_EQ(DiagnosticStatus::OK, DiagnoseGamepadLevel(true, false));
  EXPECT_EQ(DiagnosticStatus::OK, DiagnoseGamepadLevel(true, true));
}

TEST(TestDiagnoseGamepadLevel, LostGamepadWarns)
{
  EXPECT_EQ(DiagnosticStatus::WARN, DiagnoseGamepadLevel(false, true));
}

// /dev/null is a character device (1:3) on every Linux system, so it stands in for a joystick node.
class TestJoystickPresent : public testing::Test
{
protected:
  TestJoystickPresent()
  {
    base_dir_ = std::filesystem::path(testing::TempDir()) /
                ("test_joystick_present_" + std::to_string(getpid()));
    std::error_code ec;
    std::filesystem::remove_all(base_dir_, ec);
    sysfs_char_dir_ = base_dir_ / "sys_dev_char";
    std::filesystem::create_directories(sysfs_char_dir_);
  }

  ~TestJoystickPresent()
  {
    std::error_code ec;
    std::filesystem::remove_all(base_dir_, ec);
  }

  void RegisterNull() { std::filesystem::create_directories(sysfs_char_dir_ / "1:3"); }

  std::filesystem::path base_dir_;
  std::filesystem::path sysfs_char_dir_;
};

TEST_F(TestJoystickPresent, RegisteredDevice)
{
  RegisterNull();
  EXPECT_TRUE(JoystickPresent("/dev/null", sysfs_char_dir_));
}

TEST_F(TestJoystickPresent, RegisteredEntryWithHiddenTarget)
{
  // Real /sys/dev/char entries are symlinks into /sys/devices, which a sandbox may hide.
  std::filesystem::create_symlink(base_dir_ / "nowhere", sysfs_char_dir_ / "1:3");
  EXPECT_TRUE(JoystickPresent("/dev/null", sysfs_char_dir_));
}

TEST_F(TestJoystickPresent, UnregisteredDevice)
{
  // Another registered device does not count.
  std::filesystem::create_directories(sysfs_char_dir_ / "13:0");
  EXPECT_FALSE(JoystickPresent("/dev/null", sysfs_char_dir_));
}

TEST_F(TestJoystickPresent, SymlinkNamedJsIsCheckedByItsTarget)
{
  // A udev link or a remapped node: the name says nothing, the device number decides.
  const auto link = base_dir_ / "js_pad";
  std::filesystem::create_symlink("/dev/null", link);
  EXPECT_FALSE(JoystickPresent(link.string(), sysfs_char_dir_));
  RegisterNull();
  EXPECT_TRUE(JoystickPresent(link.string(), sysfs_char_dir_));
}

TEST_F(TestJoystickPresent, NonCharacterDeviceIsLeftToOpen)
{
  EXPECT_TRUE(JoystickPresent(base_dir_.string(), sysfs_char_dir_));
  EXPECT_TRUE(JoystickPresent((base_dir_ / "missing").string(), sysfs_char_dir_));

  const auto dangling = base_dir_ / "js_dangling";
  std::filesystem::create_symlink(base_dir_ / "missing", dangling);
  EXPECT_TRUE(JoystickPresent(dangling.string(), sysfs_char_dir_));
}

TEST_F(TestJoystickPresent, MissingSysfsIsLeftToOpen)
{
  EXPECT_TRUE(JoystickPresent("/dev/null", sysfs_char_dir_ / "missing"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
