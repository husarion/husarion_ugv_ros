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

#ifndef HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_UTILS_HPP_
#define HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_UTILS_HPP_

#include <sys/stat.h>
#include <sys/sysmacros.h>

#include <filesystem>
#include <string>
#include <system_error>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>

namespace husarion_ugv_teleop
{

/**
 * @brief Level of the gamepad diagnostic. A gamepad is optional, so a robot that never had one is
 * OK. A gamepad that was opened and then lost is WARN, since someone may have been driving with it.
 */
inline unsigned char DiagnoseGamepadLevel(const bool connected, const bool connected_before)
{
  if (connected || !connected_before) {
    return diagnostic_msgs::msg::DiagnosticStatus::OK;
  }
  return diagnostic_msgs::msg::DiagnosticStatus::WARN;
}

/**
 * @brief Tells whether the kernel has a device behind a joystick path, without opening it.
 *
 * A jsN node can exist with no device behind it, for example a placeholder bound into a container.
 * Opening it makes the kernel request the char-major-13 module, a synchronous modprobe on every
 * attempt. stat() follows symlinks and does not open the node, and /sys/dev/char lists a device
 * number only while a device is registered, so remapped nodes and udev links are checked by the
 * device they point at. A path that is missing or not a character device (a dangling symlink
 * included), a missing sysfs_char_dir or an entry that can not be read, is reported
 * present and left to open().
 *
 * The sysfs entry appears once joydev has registered the device, so a hot-plugged gamepad is found
 * only if something loads joydev: udev does that on the input device's modalias, or joydev is
 * built in.
 *
 * @param dev Path of the joystick device
 * @param sysfs_char_dir Directory listing the registered character devices by major:minor
 */
inline bool JoystickPresent(
  const std::string & dev, const std::filesystem::path & sysfs_char_dir = "/sys/dev/char")
{
  struct stat st;
  if (stat(dev.c_str(), &st) != 0 || !S_ISCHR(st.st_mode)) {
    return true;
  }

  std::error_code ec;
  if (!std::filesystem::is_directory(sysfs_char_dir, ec)) {
    return true;
  }
  const auto device_number = std::to_string(major(st.st_rdev)) + ":" +
                             std::to_string(minor(st.st_rdev));
  // The entry itself, not its target: a sandbox may hide /sys/devices behind the link.
  const auto entry = std::filesystem::symlink_status(sysfs_char_dir / device_number, ec);
  if (ec && ec != std::errc::no_such_file_or_directory) {
    return true;
  }
  return std::filesystem::exists(entry);
}

}  // namespace husarion_ugv_teleop

#endif  // HUSARION_UGV_TELEOP_HUSARION_UGV_TELEOP_GAMEPAD_UTILS_HPP_
