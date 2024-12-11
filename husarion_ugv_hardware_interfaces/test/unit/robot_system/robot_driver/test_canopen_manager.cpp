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

#include <memory>
#include <stdexcept>
#include <string>

#include <gtest/gtest.h>
#include <sys/socket.h>

#include "husarion_ugv_hardware_interfaces/robot_system/robot_driver/canopen_manager.hpp"

#include "utils/fake_can_socket.hpp"
#include "utils/test_constants.hpp"

#include "husarion_ugv_utils/test/test_utils.hpp"

class TestCANopenManager : public ::testing::Test
{
public:
  TestCANopenManager();

  ~TestCANopenManager() {}

protected:
  std::unique_ptr<husarion_ugv_hardware_interfaces_test::FakeCANSocket> can_socket_;
  std::unique_ptr<husarion_ugv_hardware_interfaces::CANopenManager> canopen_manager_;
};

TestCANopenManager::TestCANopenManager()
{
  can_socket_ = std::make_unique<husarion_ugv_hardware_interfaces_test::FakeCANSocket>(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings.can_interface_name);

  canopen_manager_ = std::make_unique<husarion_ugv_hardware_interfaces::CANopenManager>(
    husarion_ugv_hardware_interfaces_test::kCANopenSettings);
}

TEST_F(TestCANopenManager, InitializeAndDeinitialize)
{
  ASSERT_NO_THROW(canopen_manager_->Initialize());
  ASSERT_NO_THROW(canopen_manager_->Deinitialize());

  // Check if deinitialization worked correctly - initialize once again
  ASSERT_NO_THROW(canopen_manager_->Initialize());
}

TEST_F(TestCANopenManager, Activate)
{
  can_socket_->Initialize();

  ASSERT_NO_THROW(canopen_manager_->Initialize());
  EXPECT_NO_THROW(canopen_manager_->Activate());
  ASSERT_NO_THROW(canopen_manager_->Deinitialize());

  // Check if deinitialization worked correctly - activate once again
  ASSERT_NO_THROW(canopen_manager_->Initialize());
  EXPECT_NO_THROW(canopen_manager_->Activate());
}

TEST_F(TestCANopenManager, ActivateNotInitialized)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { canopen_manager_->Activate(); }, "CANopenManager not initialized."));
}

TEST_F(TestCANopenManager, ActivateNoCommunication)
{
  ASSERT_NO_THROW(canopen_manager_->Initialize());
  EXPECT_THROW(canopen_manager_->Activate(), std::runtime_error);
}

TEST_F(TestCANopenManager, GetMaster)
{
  can_socket_->Initialize();
  ASSERT_NO_THROW(canopen_manager_->Initialize());
  EXPECT_NO_THROW(canopen_manager_->GetMaster());
}

TEST_F(TestCANopenManager, GetMasterNotInitialized)
{
  EXPECT_TRUE(husarion_ugv_utils::test_utils::IsMessageThrown<std::runtime_error>(
    [&]() { canopen_manager_->GetMaster(); }, "CANopenManager not initialized."));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
