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

#include "husarion_ugv_battery/protection/uvp_parameters.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "yaml-cpp/yaml.h"

namespace husarion_ugv_battery
{

namespace
{

std::string Trim(const std::string & text)
{
  const auto begin = std::find_if_not(
    text.begin(), text.end(), [](unsigned char c) { return std::isspace(c); });
  const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
                     return std::isspace(c);
                   }).base();
  return begin < end ? std::string(begin, end) : std::string();
}

std::string FormatNumber(const double value)
{
  std::ostringstream stream;
  stream << value;
  return stream.str();
}

// The parameter as a number. Empty when it is unset or an empty string, error when it is not a
// number.
std::optional<double> ToNumber(const rclcpp::Parameter & parameter, std::string & error)
{
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return std::nullopt;
    case rclcpp::ParameterType::PARAMETER_DOUBLE:
      return parameter.as_double();
    case rclcpp::ParameterType::PARAMETER_INTEGER:
      return static_cast<double>(parameter.as_int());
    case rclcpp::ParameterType::PARAMETER_STRING: {
      const auto text = Trim(parameter.as_string());
      if (text.empty()) {
        return std::nullopt;
      }
      char * end = nullptr;
      const double value = std::strtod(text.c_str(), &end);
      if (end != text.c_str() + text.size()) {
        error = "'" + text + "' is not a number";
        return std::nullopt;
      }
      return value;
    }
    default:
      error = "expected a number, got a " + parameter.get_type_name();
      return std::nullopt;
  }
}

std::optional<bool> ToBool(const rclcpp::Parameter & parameter, std::string & error)
{
  switch (parameter.get_type()) {
    case rclcpp::ParameterType::PARAMETER_NOT_SET:
      return std::nullopt;
    case rclcpp::ParameterType::PARAMETER_BOOL:
      return parameter.as_bool();
    case rclcpp::ParameterType::PARAMETER_STRING: {
      auto text = Trim(parameter.as_string());
      std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
      });
      if (text.empty()) {
        return std::nullopt;
      }
      if (text == "true") {
        return true;
      }
      if (text == "false") {
        return false;
      }
      error = "'" + text + "' is not true or false";
      return std::nullopt;
    }
    default:
      error = "expected true or false, got a " + parameter.get_type_name();
      return std::nullopt;
  }
}

class UvpParameterReader
{
public:
  UvpParameterReader(rclcpp::Node & node, std::vector<UvpNote> & notes) : node_(node), notes_(notes)
  {
  }

  rclcpp::Parameter Declare(const std::string & name, const rclcpp::ParameterValue & default_value)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.dynamic_typing = true;
    descriptor.read_only = true;
    const auto full_name = "uvp." + name;
    if (!node_.has_parameter(full_name)) {
      node_.declare_parameter(full_name, default_value, descriptor);
    }
    return node_.get_parameter(full_name);
  }

  bool ReadBool(const std::string & name, const bool default_value)
  {
    const auto parameter = Declare(name, rclcpp::ParameterValue(default_value));
    std::string error;
    const auto value = ToBool(parameter, error);
    if (!error.empty()) {
      Reject(name, error, default_value ? "true" : "false");
    }
    return value.value_or(default_value);
  }

  // A number in [min, max], or one of the extra values.
  double ReadNumber(
    const std::string & name, const double default_value, const double min, const double max,
    const bool integer = false, const std::vector<double> & extra_values = {})
  {
    const auto parameter = Declare(
      name, integer ? rclcpp::ParameterValue(static_cast<int64_t>(default_value))
                    : rclcpp::ParameterValue(default_value));
    std::string error;
    const auto value = ToNumber(parameter, error);
    if (!error.empty()) {
      Reject(name, error, FormatNumber(default_value));
      return default_value;
    }
    if (!value.has_value()) {
      return default_value;
    }

    const bool extra = std::find(extra_values.begin(), extra_values.end(), value.value()) !=
                       extra_values.end();
    const bool in_range = std::isfinite(value.value()) && value.value() >= min &&
                          value.value() <= max;
    const bool whole = !integer || std::floor(value.value()) == value.value();
    if (!(extra || (in_range && whole))) {
      std::string expected = std::string(integer ? "a whole number" : "a value") + " in [" +
                             FormatNumber(min) + ", " + FormatNumber(max) + "]";
      for (const auto extra_value : extra_values) {
        expected += " or " + FormatNumber(extra_value);
      }
      Reject(
        name, FormatNumber(value.value()) + " is not " + expected, FormatNumber(default_value));
      return default_value;
    }
    return value.value();
  }

  std::string ReadString(const std::string & name, const std::string & default_value)
  {
    const auto parameter = Declare(name, rclcpp::ParameterValue(default_value));
    if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      Reject(name, "expected text, got a " + parameter.get_type_name(), "'" + default_value + "'");
      return default_value;
    }
    return parameter.as_string();
  }

  void Reject(const std::string & name, const std::string & error, const std::string & used)
  {
    notes_.push_back(
      {UvpNoteLevel::ERROR, "Parameter uvp." + name + ": " + error + ", using " + used + "."});
  }

private:
  rclcpp::Node & node_;
  std::vector<UvpNote> & notes_;
};

}  // namespace

UvpParams DeclareUvpParameters(rclcpp::Node & node)
{
  UvpParams params;
  UvpParameterReader reader(node, params.notes);

  params.enable = reader.ReadBool("enable", false);

  {
    const auto parameter = reader.Declare(
      "threshold_v", rclcpp::ParameterValue(std::numeric_limits<double>::quiet_NaN()));
    std::string error;
    const auto value = ToNumber(parameter, error);
    if (!error.empty()) {
      params.notes.push_back(
        {UvpNoteLevel::ERROR, "Parameter uvp.threshold_v: " + error + ", ignored."});
    } else if (value.has_value() && !std::isnan(value.value())) {
      if (value.value() >= uvp::kMinThreshold && value.value() <= uvp::kMaxThreshold) {
        params.threshold = static_cast<float>(value.value());
      } else {
        params.notes.push_back(
          {UvpNoteLevel::ERROR, "Parameter uvp.threshold_v: " + FormatNumber(value.value()) +
                                  " V is not in [" + FormatNumber(uvp::kMinThreshold) + ", " +
                                  FormatNumber(uvp::kMaxThreshold) + "] V, ignored."});
      }
    }
  }

  params.charge_asap_offset = static_cast<float>(reader.ReadNumber(
    "charge_asap_offset_v", uvp::kDefaultChargeAsapOffset, uvp::kMinChargeAsapOffset,
    uvp::kMaxChargeAsapOffset));
  params.consecutive_samples = static_cast<int>(reader.ReadNumber(
    "consecutive_samples", uvp::kDefaultConsecutiveSamples, uvp::kMinConsecutiveSamples,
    uvp::kMaxConsecutiveSamples, true));
  params.clear_hysteresis = static_cast<float>(reader.ReadNumber(
    "clear_hysteresis_v", uvp::kDefaultClearHysteresis, uvp::kMinClearHysteresis,
    uvp::kMaxClearHysteresis));

  {
    const auto parameter = reader.Declare("mode", rclcpp::ParameterValue(std::string("STOP")));
    auto mode = parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING
                  ? Trim(parameter.as_string())
                  : std::string();
    std::transform(mode.begin(), mode.end(), mode.begin(), [](unsigned char c) {
      return static_cast<char>(std::toupper(c));
    });
    if (mode == "SHUTDOWN") {
      params.mode = uvp::Mode::SHUTDOWN;
    } else if (mode != "STOP" && !mode.empty()) {
      reader.Reject("mode", "'" + mode + "' is not STOP or SHUTDOWN", "STOP");
    } else if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
      reader.Reject("mode", "expected STOP or SHUTDOWN", "STOP");
    }
  }

  params.stop_timeout_min = static_cast<int>(reader.ReadNumber(
    "stop_timeout_min", uvp::kDefaultStopTimeoutMin, uvp::kMinStopTimeoutMin,
    uvp::kMaxStopTimeoutMin, true, {static_cast<double>(uvp::kReactionsOffStopTimeoutMin)}));
  params.aux_off_at_trip = reader.ReadBool("aux_off_at_trip", false);
  params.drive_window_s = static_cast<float>(reader.ReadNumber(
    "drive_window_s", uvp::kDefaultDriveWindow, uvp::kMinDriveWindow, uvp::kMaxDriveWindow));
  params.shutdown_hosts_timeout_s = static_cast<float>(reader.ReadNumber(
    "shutdown_hosts_timeout_s", uvp::kDefaultShutdownHostsTimeout, uvp::kMinShutdownHostsTimeout,
    uvp::kMaxShutdownHostsTimeout));
  params.charging_debounce_s = static_cast<float>(reader.ReadNumber(
    "charging_debounce_s", uvp::kDefaultChargingDebounce, uvp::kMinChargingDebounce,
    uvp::kMaxChargingDebounce));
  params.hat_eeprom_path = reader.ReadString("hat_eeprom_path", uvp::kDefaultHatEepromPath);
  if (params.hat_eeprom_path.empty()) {
    params.hat_eeprom_path = uvp::kDefaultHatEepromPath;
  }
  params.latch_path = reader.ReadString("latch_path", "");

  return params;
}

std::optional<std::string> ReadTextFile(const std::string & path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return std::nullopt;
  }
  std::ostringstream content;
  content << file.rdbuf();
  if (file.bad()) {
    return std::nullopt;
  }
  return content.str();
}

UvpThreshold ResolveUvpThreshold(
  const std::optional<float> & param_threshold, const std::string & hat_eeprom_path,
  const std::function<std::optional<std::string>(const std::string &)> & read_file)
{
  UvpThreshold threshold;

  if (param_threshold.has_value()) {
    threshold.value = param_threshold.value();
    threshold.source = "param";
    threshold.notes.push_back(
      {UvpNoteLevel::WARN, "UVP threshold overridden by the uvp.threshold_v parameter."});
    return threshold;
  }

  const auto content = read_file(hat_eeprom_path);
  if (!content.has_value()) {
    threshold.notes.push_back(
      {UvpNoteLevel::INFO, "No HAT EEPROM configuration at " + hat_eeprom_path + "."});
  } else {
    // A device tree string property ends with a NUL.
    std::string text = content.value();
    text.erase(std::remove(text.begin(), text.end(), '\0'), text.end());

    try {
      const auto root = YAML::Load(text);
      const auto value_node = root.IsMap() && root["robot"] && root["robot"].IsMap() &&
                                  root["robot"]["battery"] && root["robot"]["battery"].IsMap()
                                ? root["robot"]["battery"]["uvp_threshold_v"]
                                : YAML::Node();
      if (!value_node || value_node.IsNull()) {
        threshold.notes.push_back(
          {UvpNoteLevel::INFO,
           std::string("HAT EEPROM has no ") + uvp::kEepromThresholdKey + " key."});
      } else {
        const double value = value_node.as<double>();
        if (std::isfinite(value) && value >= uvp::kMinThreshold && value <= uvp::kMaxThreshold) {
          threshold.value = static_cast<float>(value);
          threshold.source = "eeprom";
          return threshold;
        }
        threshold.notes.push_back(
          {UvpNoteLevel::ERROR, std::string("HAT EEPROM ") + uvp::kEepromThresholdKey + " " +
                                  FormatNumber(value) + " V is not in [" +
                                  FormatNumber(uvp::kMinThreshold) + ", " +
                                  FormatNumber(uvp::kMaxThreshold) + "] V, ignored."});
      }
    } catch (const YAML::Exception & e) {
      threshold.notes.push_back(
        {UvpNoteLevel::ERROR,
         std::string("HAT EEPROM ") + uvp::kEepromThresholdKey + " cannot be read: " + e.what()});
    }
  }

  threshold.value = uvp::kDefaultThreshold;
  threshold.source = "default";
  return threshold;
}

}  // namespace husarion_ugv_battery
