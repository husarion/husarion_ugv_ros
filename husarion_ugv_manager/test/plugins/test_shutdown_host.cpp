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
#include "utils/plugin_test_utils.hpp"

class ShutdownHostWrapper : public husarion_ugv_manager::ShutdownHost
{
public:
  ShutdownHostWrapper() : husarion_ugv_manager::ShutdownHost() {}
  ShutdownHostWrapper(
    const std::string ip, const std::string & port = "3003", const std::string secret = "husarion",
    const float timeout = 5.0)
  : ShutdownHost(ip, port, secret, timeout)
  {
  }

  ~ShutdownHostWrapper() {}

  bool IsAvailable() { return ShutdownHost::IsAvailable(); }

  std::int64_t GetTimeSinceEpoch() { return ShutdownHost::GetTimeSinceEpoch(); }
};

class TestShutdownHost : public testing::Test
{
public:
  TestShutdownHost()
  {
    http_server_ = std::make_unique<husarion_ugv_manager::plugin_test_utils::HTTPServer>();
  }

  ~TestShutdownHost() {}

  void CreateShutdownHost(
    const std::string & ip, const std::string & port, const std::string & secret,
    const float timeout)
  {
    shutdown_host_ = std::make_unique<ShutdownHostWrapper>(ip, port, secret, timeout);
  }

  void CallUntilFinished()
  {
    auto state = this->shutdown_host_->GetState();
    while (state != husarion_ugv_manager::ShutdownHostState::SUCCESS &&
           state != husarion_ugv_manager::ShutdownHostState::FAILURE &&
           state != husarion_ugv_manager::ShutdownHostState::SKIPPED) {
      this->shutdown_host_->Call();
      state = this->shutdown_host_->GetState();
    }
  }

  bool ContainsExpression(const std::string & msg, const std::string & expression)
  {
    return msg.find(expression) != std::string::npos;
  }

  static constexpr char kDefaultServerIP[] = "21.3.7.147";
  static constexpr char kDefaultServerPort[] = "8080";

protected:
  std::unique_ptr<ShutdownHostWrapper> shutdown_host_;
  std::unique_ptr<husarion_ugv_manager::plugin_test_utils::HTTPServer> http_server_;
};

TEST_F(TestShutdownHost, GoodCheckIsAvailable)
{
  CreateShutdownHost("127.0.0.1", "8080", "husarion", 0.1);
  EXPECT_TRUE(this->shutdown_host_->IsAvailable());
  EXPECT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);
}

TEST_F(TestShutdownHost, WrongCheckIsAvailable)
{
  CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1);
  EXPECT_FALSE(this->shutdown_host_->IsAvailable());
}

TEST_F(TestShutdownHost, GetTimeSinceEpoch)
{
  CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1);
  const auto time_now = std::chrono::duration_cast<std::chrono::seconds>(
                          std::chrono::system_clock::now().time_since_epoch())
                          .count();
  auto diff = abs(time_now - this->shutdown_host_->GetTimeSinceEpoch());
  EXPECT_TRUE(diff < 2);
}

TEST_F(TestShutdownHost, HTTPServerNotAvailable)
{
  // Use localhost so host can be pinged
  ASSERT_NO_THROW(this->CreateShutdownHost("localhost", kDefaultServerPort, "husarion", 0.1));

  ASSERT_TRUE(this->shutdown_host_->IsAvailable());
  ASSERT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);

  this->shutdown_host_->Call();
  ASSERT_EQ(
    this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED);

  // Wait for response
  while (this->shutdown_host_->GetState() ==
         husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED) {
    this->shutdown_host_->Call();
  }

  ASSERT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::FAILURE);
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetError(), "Command return code:"));
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "000"));
}

TEST_F(TestShutdownHost, GoodShutdownExecute)
{
  ASSERT_NO_THROW(this->http_server_->CreateServer(kDefaultServerIP, kDefaultServerPort));
  ASSERT_NO_THROW(this->CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1));

  ASSERT_TRUE(this->shutdown_host_->IsAvailable());
  ASSERT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::IDLE);

  this->shutdown_host_->Call();
  ASSERT_EQ(
    this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED);

  // Wait for response
  while (this->shutdown_host_->GetState() ==
         husarion_ugv_manager::ShutdownHostState::COMMAND_EXECUTED) {
    this->shutdown_host_->Call();
  }
  ASSERT_EQ(
    this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::RESPONSE_RECEIVED);

  this->shutdown_host_->Call();
  ASSERT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::PINGING);

  this->shutdown_host_->Call();
  EXPECT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::SUCCESS);

  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "200"));
}

TEST_F(TestShutdownHost, HTTPServerReturnSuccessWithMessage)
{
  ASSERT_NO_THROW(this->http_server_->CreateServer(
    kDefaultServerIP, kDefaultServerPort,
    "200 OK\\r\\nContent-Type: text/plain\\r\\n\\r\\nSuccess"));
  ASSERT_NO_THROW(this->CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1));

  this->CallUntilFinished();
  EXPECT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::SUCCESS);
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "Success"));
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "200"));
}

TEST_F(TestShutdownHost, HTTPServerReturnFailure)
{
  ASSERT_NO_THROW(this->http_server_->CreateServer(
    kDefaultServerIP, kDefaultServerPort,
    "401 Unauthorized\\r\\nContent-Type: text/plain\\r\\n\\r\\nUnauthorized"));
  ASSERT_NO_THROW(this->CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1));

  this->CallUntilFinished();
  EXPECT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::FAILURE);
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "401"));
  EXPECT_TRUE(this->ContainsExpression(
    this->shutdown_host_->GetError(), "Failed to shutdown remote host. Server return code:"));
}

TEST_F(TestShutdownHost, PingHostTimeout)
{
  ASSERT_NO_THROW(
    this->http_server_->CreateServer(kDefaultServerIP, kDefaultServerPort, "200 OK", 1.0, false));
  ASSERT_NO_THROW(this->CreateShutdownHost(kDefaultServerIP, kDefaultServerPort, "husarion", 0.1));

  this->CallUntilFinished();
  EXPECT_EQ(this->shutdown_host_->GetState(), husarion_ugv_manager::ShutdownHostState::FAILURE);
  EXPECT_TRUE(this->ContainsExpression(this->shutdown_host_->GetOutput(), "200"));
  EXPECT_TRUE(this->ContainsExpression(
    this->shutdown_host_->GetError(), "Timeout waiting for host to shutdown"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
