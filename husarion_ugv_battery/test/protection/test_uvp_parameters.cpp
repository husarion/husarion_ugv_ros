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

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_battery/protection/uvp_parameters.hpp"

using husarion_ugv_battery::UvpNoteLevel;
using husarion_ugv_battery::UvpParams;

namespace uvp = husarion_ugv_battery::uvp;

class TestUvpParameters : public testing::Test
{
protected:
  UvpParams Read(const std::vector<rclcpp::Parameter> & overrides)
  {
    rclcpp::NodeOptions options;
    options.parameter_overrides(overrides);
    auto node = std::make_shared<rclcpp::Node>("uvp_parameters_test", options);
    return husarion_ugv_battery::DeclareUvpParameters(*node);
  }

  static bool HasError(
    const std::vector<husarion_ugv_battery::UvpNote> & notes, const std::string & text)
  {
    for (const auto & note : notes) {
      if (note.level == UvpNoteLevel::ERROR && note.text.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }
};

TEST_F(TestUvpParameters, Defaults)
{
  const auto params = Read({});
  EXPECT_FALSE(params.enable);
  EXPECT_TRUE(params.reactions_enabled);
  EXPECT_FALSE(params.ReactionsOffByConfig());
  EXPECT_FALSE(params.threshold.has_value());
  EXPECT_FLOAT_EQ(0.5f, params.charge_asap_offset);
  EXPECT_EQ(2, params.consecutive_samples);
  EXPECT_FLOAT_EQ(0.5f, params.clear_hysteresis);
  EXPECT_EQ(uvp::Mode::STOP, params.mode);
  EXPECT_EQ(5, params.stop_timeout_min);
  EXPECT_FALSE(params.aux_off_at_trip);
  EXPECT_FLOAT_EQ(60.0f, params.drive_window_s);
  EXPECT_FLOAT_EQ(10.0f, params.shutdown_hosts_timeout_s);
  EXPECT_FLOAT_EQ(5.0f, params.charging_debounce_s);
  EXPECT_EQ("/proc/device-tree/hat/custom_0", params.hat_eeprom_path);
  EXPECT_EQ("", params.latch_path);
  EXPECT_TRUE(params.notes.empty());
}

TEST_F(TestUvpParameters, TypedValues)
{
  const auto params = Read({
    rclcpp::Parameter("uvp.enable", true),
    rclcpp::Parameter("uvp.reactions_enabled", false),
    rclcpp::Parameter("uvp.threshold_v", 32.0),
    rclcpp::Parameter("uvp.charge_asap_offset_v", 1.0),
    rclcpp::Parameter("uvp.consecutive_samples", 4),
    rclcpp::Parameter("uvp.clear_hysteresis_v", 1.5),
    rclcpp::Parameter("uvp.mode", "SHUTDOWN"),
    rclcpp::Parameter("uvp.stop_timeout_min", 998),
    rclcpp::Parameter("uvp.aux_off_at_trip", true),
    rclcpp::Parameter("uvp.drive_window_s", 120),
    rclcpp::Parameter("uvp.shutdown_hosts_timeout_s", 3.0),
    rclcpp::Parameter("uvp.charging_debounce_s", 2.0),
    rclcpp::Parameter("uvp.latch_path", "/run/uvp_latch"),
  });
  EXPECT_TRUE(params.enable);
  EXPECT_FALSE(params.reactions_enabled);
  ASSERT_TRUE(params.threshold.has_value());
  EXPECT_FLOAT_EQ(32.0f, params.threshold.value());
  EXPECT_FLOAT_EQ(1.0f, params.charge_asap_offset);
  EXPECT_EQ(4, params.consecutive_samples);
  EXPECT_FLOAT_EQ(1.5f, params.clear_hysteresis);
  EXPECT_EQ(uvp::Mode::SHUTDOWN, params.mode);
  EXPECT_EQ(998, params.stop_timeout_min);
  EXPECT_TRUE(params.aux_off_at_trip);
  EXPECT_FLOAT_EQ(120.0f, params.drive_window_s);
  EXPECT_FLOAT_EQ(3.0f, params.shutdown_hosts_timeout_s);
  EXPECT_FLOAT_EQ(2.0f, params.charging_debounce_s);
  EXPECT_EQ("/run/uvp_latch", params.latch_path);
  EXPECT_TRUE(params.notes.empty());
}

// Launch arguments arrive as text.
TEST_F(TestUvpParameters, TextValues)
{
  const auto params = Read({
    rclcpp::Parameter("uvp.enable", "True"),
    rclcpp::Parameter("uvp.reactions_enabled", "False"),
    rclcpp::Parameter("uvp.threshold_v", "30.5"),
    rclcpp::Parameter("uvp.consecutive_samples", "3"),
    rclcpp::Parameter("uvp.mode", "shutdown"),
    rclcpp::Parameter("uvp.stop_timeout_min", "10"),
    rclcpp::Parameter("uvp.aux_off_at_trip", "false"),
    rclcpp::Parameter("uvp.drive_window_s", " 30 "),
  });
  EXPECT_TRUE(params.enable);
  EXPECT_FALSE(params.reactions_enabled);
  ASSERT_TRUE(params.threshold.has_value());
  EXPECT_FLOAT_EQ(30.5f, params.threshold.value());
  EXPECT_EQ(3, params.consecutive_samples);
  EXPECT_EQ(uvp::Mode::SHUTDOWN, params.mode);
  EXPECT_EQ(10, params.stop_timeout_min);
  EXPECT_FALSE(params.aux_off_at_trip);
  EXPECT_FLOAT_EQ(30.0f, params.drive_window_s);
  EXPECT_TRUE(params.notes.empty());
}

TEST_F(TestUvpParameters, EmptyThresholdIsUnset)
{
  const auto params = Read({rclcpp::Parameter("uvp.threshold_v", "")});
  EXPECT_FALSE(params.threshold.has_value());
  EXPECT_TRUE(params.notes.empty());
}

TEST_F(TestUvpParameters, OutOfRangeThresholdIsIgnored)
{
  for (const double value : {27.9, 36.1}) {
    const auto params = Read({rclcpp::Parameter("uvp.threshold_v", value)});
    EXPECT_FALSE(params.threshold.has_value());
    EXPECT_TRUE(HasError(params.notes, "uvp.threshold_v"));
  }
  for (const double value : {28.0, 36.0}) {
    const auto params = Read({rclcpp::Parameter("uvp.threshold_v", value)});
    EXPECT_TRUE(params.threshold.has_value());
  }
}

TEST_F(TestUvpParameters, InvalidValuesKeepTheDefaults)
{
  const auto params = Read({
    rclcpp::Parameter("uvp.reactions_enabled", "maybe"),
    rclcpp::Parameter("uvp.threshold_v", "abc"),
    rclcpp::Parameter("uvp.charge_asap_offset_v", 0.1),
    rclcpp::Parameter("uvp.consecutive_samples", 2.5),
    rclcpp::Parameter("uvp.clear_hysteresis_v", 2.1),
    rclcpp::Parameter("uvp.mode", "HALT"),
    rclcpp::Parameter("uvp.stop_timeout_min", 61),
    rclcpp::Parameter("uvp.aux_off_at_trip", "maybe"),
    rclcpp::Parameter("uvp.drive_window_s", 9.0),
    rclcpp::Parameter("uvp.shutdown_hosts_timeout_s", 31.0),
    rclcpp::Parameter("uvp.charging_debounce_s", "fast"),
  });
  EXPECT_TRUE(params.reactions_enabled);
  EXPECT_FALSE(params.threshold.has_value());
  EXPECT_FLOAT_EQ(uvp::kDefaultChargeAsapOffset, params.charge_asap_offset);
  EXPECT_EQ(uvp::kDefaultConsecutiveSamples, params.consecutive_samples);
  EXPECT_FLOAT_EQ(uvp::kDefaultClearHysteresis, params.clear_hysteresis);
  EXPECT_EQ(uvp::Mode::STOP, params.mode);
  EXPECT_EQ(uvp::kDefaultStopTimeoutMin, params.stop_timeout_min);
  EXPECT_FALSE(params.aux_off_at_trip);
  EXPECT_FLOAT_EQ(uvp::kDefaultDriveWindow, params.drive_window_s);
  EXPECT_FLOAT_EQ(uvp::kDefaultShutdownHostsTimeout, params.shutdown_hosts_timeout_s);
  EXPECT_FLOAT_EQ(uvp::kDefaultChargingDebounce, params.charging_debounce_s);
  for (const auto * name :
       {"reactions_enabled", "threshold_v", "charge_asap_offset_v", "consecutive_samples",
        "clear_hysteresis_v", "mode", "stop_timeout_min", "aux_off_at_trip", "drive_window_s",
        "shutdown_hosts_timeout_s", "charging_debounce_s"}) {
    EXPECT_TRUE(HasError(params.notes, std::string("uvp.") + name)) << name;
  }
}

TEST_F(TestUvpParameters, ReactionsOffByTheBoolOrTheLegacyStopTimeout)
{
  EXPECT_TRUE(Read({rclcpp::Parameter("uvp.reactions_enabled", false)}).ReactionsOffByConfig());
  EXPECT_TRUE(Read({rclcpp::Parameter("uvp.stop_timeout_min", 998)}).ReactionsOffByConfig());
  EXPECT_FALSE(Read({rclcpp::Parameter("uvp.stop_timeout_min", 60)}).ReactionsOffByConfig());
  EXPECT_FALSE(Read({rclcpp::Parameter("uvp.reactions_enabled", "true")}).ReactionsOffByConfig());
}

TEST_F(TestUvpParameters, StopTimeoutAcceptsOnlyTheReactionsOffValueOutsideItsRange)
{
  EXPECT_EQ(998, Read({rclcpp::Parameter("uvp.stop_timeout_min", 998)}).stop_timeout_min);
  EXPECT_EQ(5, Read({rclcpp::Parameter("uvp.stop_timeout_min", 997)}).stop_timeout_min);
  EXPECT_EQ(5, Read({rclcpp::Parameter("uvp.stop_timeout_min", 999)}).stop_timeout_min);
  EXPECT_EQ(5, Read({rclcpp::Parameter("uvp.stop_timeout_min", 0)}).stop_timeout_min);
  EXPECT_EQ(60, Read({rclcpp::Parameter("uvp.stop_timeout_min", 60)}).stop_timeout_min);
  EXPECT_EQ(1, Read({rclcpp::Parameter("uvp.stop_timeout_min", "1")}).stop_timeout_min);
}

class TestUvpThreshold : public testing::Test
{
protected:
  std::optional<std::string> ReadFile(const std::string & path)
  {
    read_paths_.push_back(path);
    const auto it = files_.find(path);
    if (it == files_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  husarion_ugv_battery::UvpThreshold Resolve(const std::optional<float> & param)
  {
    return husarion_ugv_battery::ResolveUvpThreshold(
      param, kEeprom, [this](const std::string & path) { return ReadFile(path); });
  }

  static bool Has(
    const husarion_ugv_battery::UvpThreshold & threshold, const UvpNoteLevel level,
    const std::string & text)
  {
    for (const auto & note : threshold.notes) {
      if (note.level == level && note.text.find(text) != std::string::npos) {
        return true;
      }
    }
    return false;
  }

  static constexpr char kEeprom[] = "/proc/device-tree/hat/custom_0";
  std::map<std::string, std::string> files_;
  std::vector<std::string> read_paths_;
};

TEST_F(TestUvpThreshold, ParamWinsOverEeprom)
{
  files_[kEeprom] = "robot:\n  battery:\n    uvp_threshold_v: 30.0\n";
  const auto threshold = Resolve(32.0f);
  EXPECT_FLOAT_EQ(32.0f, threshold.value);
  EXPECT_EQ("param", threshold.source);
  EXPECT_TRUE(Has(threshold, UvpNoteLevel::WARN, "overridden"));
}

TEST_F(TestUvpThreshold, EepromWinsOverDefault)
{
  // A device tree string property ends with a NUL.
  files_[kEeprom] = std::string("robot:\n  model: LNX\n  battery:\n    uvp_threshold_v: 30.2\n") +
                    '\0';
  const auto threshold = Resolve(std::nullopt);
  EXPECT_FLOAT_EQ(30.2f, threshold.value);
  EXPECT_EQ("eeprom", threshold.source);
  EXPECT_TRUE(threshold.notes.empty());
}

TEST_F(TestUvpThreshold, MissingFileIsDefaultWithInfo)
{
  const auto threshold = Resolve(std::nullopt);
  EXPECT_FLOAT_EQ(31.0f, threshold.value);
  EXPECT_EQ("default", threshold.source);
  EXPECT_TRUE(Has(threshold, UvpNoteLevel::INFO, "No HAT EEPROM"));
}

TEST_F(TestUvpThreshold, MissingKeyIsDefaultWithInfo)
{
  files_[kEeprom] = "robot:\n  model: PTH\n";
  const auto threshold = Resolve(std::nullopt);
  EXPECT_EQ("default", threshold.source);
  EXPECT_TRUE(Has(threshold, UvpNoteLevel::INFO, "robot.battery.uvp_threshold_v"));
}

TEST_F(TestUvpThreshold, InvalidEepromValueIsDefaultWithError)
{
  for (const auto * value : {"abc", "40.0", "27.0", ".nan"}) {
    files_[kEeprom] = std::string("robot:\n  battery:\n    uvp_threshold_v: ") + value + "\n";
    const auto threshold = Resolve(std::nullopt);
    EXPECT_FLOAT_EQ(31.0f, threshold.value) << value;
    EXPECT_EQ("default", threshold.source) << value;
    EXPECT_TRUE(Has(threshold, UvpNoteLevel::ERROR, "uvp_threshold_v")) << value;
  }
}

TEST_F(TestUvpThreshold, ReadsOnlyTheHatEeprom)
{
  const auto threshold = Resolve(std::nullopt);
  EXPECT_EQ("default", threshold.source);
  ASSERT_FALSE(read_paths_.empty());
  for (const auto & path : read_paths_) {
    EXPECT_EQ(kEeprom, path);
  }
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
