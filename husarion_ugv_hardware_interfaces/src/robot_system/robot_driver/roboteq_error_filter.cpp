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

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/roboteq_error_filter.hpp"

#include <algorithm>
#include <atomic>
#include <map>

namespace husarion_ugv_hardware_interfaces
{

void ErrorFilter::UpdateError(const bool current_error)
{
  if (current_error) {
    ++current_error_count_;
    if (current_error_count_ >= max_error_count_) {
      error_ = true;
    }
  } else {
    current_error_count_ = 0;
  }
}

void ErrorFilter::ClearError()
{
  error_ = false;
  current_error_count_ = 0;
}

RoboteqErrorFilter::RoboteqErrorFilter(
  const unsigned max_write_pdo_cmds_errors_count,
  const unsigned max_read_pdo_motor_states_errors_count,
  const unsigned max_read_pdo_driver_state_errors_count,
  const unsigned max_roboteq_driver_error_count)
{
  error_filters_.insert(
    {ErrorsFilterIds::WRITE_PDO_CMDS, ErrorFilter(max_write_pdo_cmds_errors_count)});
  error_filters_.insert(
    {ErrorsFilterIds::READ_PDO_MOTOR_STATES, ErrorFilter(max_read_pdo_motor_states_errors_count)});
  error_filters_.insert(
    {ErrorsFilterIds::READ_PDO_DRIVER_STATE, ErrorFilter(max_read_pdo_driver_state_errors_count)});
  error_filters_.insert(
    {ErrorsFilterIds::ROBOTEQ_DRIVER, ErrorFilter(max_roboteq_driver_error_count)});
}

bool RoboteqErrorFilter::IsError() const
{
  return std::any_of(error_filters_.begin(), error_filters_.end(), [](const auto & filter) {
    return filter.second.IsError();
  });
};

void RoboteqErrorFilter::UpdateError(const ErrorsFilterIds id, const bool current_error)
{
  ClearErrorsIfFlagSet();
  error_filters_.at(id).UpdateError(current_error);
}

std::map<std::string, bool> RoboteqErrorFilter::GetErrorMap() const
{
  std::map<std::string, bool> error_map;

  std::for_each(error_filters_.begin(), error_filters_.end(), [&error_map](const auto & filter) {
    auto error_name = error_filter_id_names.at(filter.first);
    auto error_value = filter.second.IsError();
    error_map.emplace(error_name, error_value);
  });

  return error_map;
}

void RoboteqErrorFilter::ClearErrorsIfFlagSet()
{
  if (clear_errors_) {
    std::for_each(error_filters_.begin(), error_filters_.end(), [](auto & filter) {
      filter.second.ClearError();
    });
    clear_errors_.store(false);
  }
}

}  // namespace husarion_ugv_hardware_interfaces
