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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CMD_WRITE_STATUS_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CMD_WRITE_STATUS_HPP_

#include <array>
#include <atomic>
#include <cstddef>
#include <mutex>
#include <stdexcept>
#include <string>

namespace husarion_ugv_hardware_interfaces
{

/**
 * @brief Result of the newest completed velocity command write, per channel. A failure stays
 * reported until a later write on the same channel succeeds, so a cycle whose write has not run
 * yet does not count as a success.
 */
class CmdWriteStatus
{
public:
  void RecordSuccess(const std::size_t slot) { failed_.at(slot).store(false); }

  void RecordFailure(const std::size_t slot, const std::string & error)
  {
    {
      std::lock_guard<std::mutex> lck(mtx_);
      errors_.at(slot) = error;
    }
    failed_.at(slot).store(true);
  }

  /**
   * @exception std::runtime_error if the newest completed write on this channel failed
   */
  void ThrowIfFailed(const std::size_t slot) const
  {
    if (failed_.at(slot).load()) {
      std::lock_guard<std::mutex> lck(mtx_);
      throw std::runtime_error("Velocity command write failed, " + errors_.at(slot));
    }
  }

private:
  std::array<std::atomic_bool, 2> failed_{};
  mutable std::mutex mtx_;
  std::array<std::string, 2> errors_;
};

}  // namespace husarion_ugv_hardware_interfaces

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_HUSARION_UGV_HARDWARE_INTERFACES_ROBOT_SYSTEM_ROBOT_DRIVER_CMD_WRITE_STATUS_HPP_
