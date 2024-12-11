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

#include <string>

#include "gtest/gtest.h"

#include "husarion_ugv_manager/plugins/shutdown_host.hpp"

class TestShutdownHost : public testing::Test
{
public:
  void Create(
    const std::string ip, const std::string user, const int port, const std::string command,
    const float timeout, const bool ping_for_success);
  bool IsAvailable();
  void Call();
  husarion_ugv_manager::ShutdownHostState GetState() const;
  std::string GetResponse() const;

private:
  std::unique_ptr<husarion_ugv_manager::ShutdownHost> shutdown_host;
};

void TestShutdownHost::Create(
  const std::string ip, const std::string user, const int port, const std::string command,
  const float timeout, const bool ping_for_success)
{
  shutdown_host = std::make_unique<husarion_ugv_manager::ShutdownHost>(
    ip, user, port, command, timeout, ping_for_success);
}

bool TestShutdownHost::IsAvailable() { return shutdown_host->IsAvailable(); }

void TestShutdownHost::Call() { shutdown_host->Call(); }

husarion_ugv_manager::ShutdownHostState TestShutdownHost::GetState() const
{
  return shutdown_host->GetState();
}

std::string TestShutdownHost::GetResponse() const { return shutdown_host->GetResponse(); }

TEST_F(TestShutdownHost, GoodCheckIsAvailable)
{
  Create("127.0.0.1", "husarion", 22, "echo HelloWorld", 0.1, true);
  EXPECT_TRUE(IsAvailable());
  EXPECT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);
}

TEST_F(TestShutdownHost, WrongCheckIsAvailable)
{
  Create("1.45.23.26", "husarion", 22, "echo HelloWorld", 0.1, true);
  EXPECT_FALSE(IsAvailable());
}

TEST_F(TestShutdownHost, GoodCommandExecute)
{
  Create("127.0.0.1", "husarion", 22, "echo HelloWorld", 0.1, false);

  ASSERT_TRUE(IsAvailable());
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);
  Call();
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED);
  // Wait for response
  while (GetState() == husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED) {
    Call();
  }
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::RESPONSE_RECEIVED);

  const auto response = GetResponse();
  EXPECT_EQ(response, "HelloWorld\n");

  Call();

  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::PINGING);
  Call();
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::SUCCESS);
}

TEST_F(TestShutdownHost, WrongHostPing)
{
  Create("1.45.23.26", "husarion", 22, "echo HelloWorld", 0.1, true);

  EXPECT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);
  Call();
  EXPECT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::SKIPPED);
}

TEST_F(TestShutdownHost, CheckTimeout)
{
  Create("127.0.0.1", "husarion", 22, "sleep 0.2", 0.1, false);

  ASSERT_TRUE(IsAvailable());
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);
  Call();
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED);
  // Wait for response
  while (GetState() == husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED) {
    Call();
  }
  ASSERT_EQ(GetState(), husarion_ugv_manager::ShutdownHostState::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
