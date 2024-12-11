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

#ifndef HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_FAKE_CAN_SOCKET_HPP_
#define HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_FAKE_CAN_SOCKET_HPP_

#include <stdexcept>
#include <string>

#include <sys/socket.h>

namespace husarion_ugv_hardware_interfaces_test
{

class FakeCANSocket
{
public:
  FakeCANSocket(const std::string & can_interface_name)
  : can_interface_name_(can_interface_name), can_device_created_(false)
  {
  }

  ~FakeCANSocket() { Deinitialize(); }

  void Initialize()
  {
    if (system("sudo modprobe vcan") != 0) {
      throw std::runtime_error("Failed to load vcan module");
    }

    // if link already exists, do not create it
    const auto check_command = "ip link show " + can_interface_name_ + " > /dev/null 2>&1";
    if (std::system(check_command.c_str()) != 0) {
      const auto command = "sudo ip link add dev " + can_interface_name_ + " type vcan";
      if (system(command.c_str()) != 0) {
        throw std::runtime_error("Failed to add vcan device");
      }
    }

    can_device_created_ = true;

    const auto command = "sudo ip link set up " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to set up vcan device");
    }
  }

  void Deinitialize()
  {
    if (!can_device_created_) {
      return;
    }

    std::string command = "sudo ip link delete " + can_interface_name_;
    if (system(command.c_str()) != 0) {
      throw std::runtime_error("Failed to delete vcan device");
    }

    can_device_created_ = false;
  }

private:
  const std::string can_interface_name_;
  bool can_device_created_;
};

}  // namespace husarion_ugv_hardware_interfaces_test

#endif  // HUSARION_UGV_HARDWARE_INTERFACES_TEST_UTILS_FAKE_CAN_SOCKET_HPP_
