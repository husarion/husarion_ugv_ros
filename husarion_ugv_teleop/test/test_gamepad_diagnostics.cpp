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

#include <gtest/gtest.h>

#include "husarion_ugv_teleop/gamepad_diagnostics.hpp"

using diagnostic_msgs::msg::DiagnosticStatus;
using husarion_ugv_teleop::DiagnoseGamepad;

TEST(TestGamepadDiagnostics, NoGamepadIsOK)
{
  const auto diagnostic = DiagnoseGamepad(false, false);
  EXPECT_EQ(DiagnosticStatus::OK, diagnostic.level);
  EXPECT_EQ("No gamepad connected (optional).", diagnostic.message);
}

TEST(TestGamepadDiagnostics, ConnectedGamepadIsOK)
{
  EXPECT_EQ(DiagnosticStatus::OK, DiagnoseGamepad(true, false).level);
  EXPECT_EQ(DiagnosticStatus::OK, DiagnoseGamepad(true, true).level);
}

TEST(TestGamepadDiagnostics, LostGamepadWarns)
{
  const auto diagnostic = DiagnoseGamepad(false, true);
  EXPECT_EQ(DiagnosticStatus::WARN, diagnostic.level);
  EXPECT_EQ("Gamepad disconnected.", diagnostic.message);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
