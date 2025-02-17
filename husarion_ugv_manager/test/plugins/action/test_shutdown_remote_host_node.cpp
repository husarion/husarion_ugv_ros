// Copyright 2025 Husarion sp. z o.o.
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

#include <chrono>
#include <cstdint>
#include <map>
#include <string>
#include <thread>

#include "gtest/gtest.h"

#include "behaviortree_cpp/bt_factory.h"

#include "husarion_ugv_manager/plugins/action/shutdown_remote_host_node.hpp"
#include "utils/plugin_test_utils.hpp"

class TestShutdownRemoteHost : public husarion_ugv_manager::plugin_test_utils::PluginTestUtils
{
public:
  TestShutdownRemoteHost() : PluginTestUtils() {}

  ~TestShutdownRemoteHost()
  {
    if (server_thread_.joinable()) {
      server_thread_.join();
    }

    StopDummyHost();
  }

  /**
   * @brief Creates a simple HTTP server that responds with a specified message. This function
   * spawns a new thread to run a command that sets up a basic HTTP server using netcat. The server
   * will respond with the specified HTTP response message and will run for a limited time.
   *
   * @param server_ip The IP address of the server.
   * @param server_port The port number on which the server will listen.
   * @param response The HTTP response message to be sent by the server. Default is "200 OK".
   * @param timeout The duration (in seconds) for which the server will run. Default is 1.0 seconds.
   *
   * @throws std::runtime_error if the server fails to start.
   */
  void CreateServer(
    const std::string & server_ip, const std::string & server_port,
    const std::string & response = "200 OK", const float timeout = 1.0,
    const bool stop_dummy_host = true)
  {
    // Command to echo an HTTP response to netcat
    std::string command = "echo -e 'HTTP/1.1 " + response + "' | nc -l -q 0 -s " + server_ip +
                          " -p " + server_port;
    const auto bash_cmd = "timeout " + std::to_string(timeout) + " bash -c \"" + command +
                          "\" >> /dev/null 2>&1";

    StartDummyHost();

    server_thread_ = std::thread([&, stop_dummy_host]() {
      ExecuteSystemCommand(bash_cmd);
      if (stop_dummy_host) {
        StopDummyHost();
      }
      server_thread_finished_ = true;
    });

    // Wait for the server to start
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  /**
   * @brief Starts a dummy host for testing purposes.
   *
   * @throws std::runtime_error if the command fails to start dummy host.
   */
  void StartDummyHost()
  {
    ExecuteSystemCommand("sudo ip link add " + std::string(kDummyInterfaceName) + " type dummy");
    ExecuteSystemCommand(
      "sudo ip addr add " + std::string(kDefaultServerIP) + " dev " +
      std::string(kDummyInterfaceName) + "");
    ExecuteSystemCommand("sudo ip link set " + std::string(kDummyInterfaceName) + " up");
  }

  /**
   * @brief Stops the dummy host. This command will never throw an exception as it assumes that the
   * dummy host may have never be started.
   */
  void StopDummyHost()
  {
    ExecuteSystemCommand(
      "sudo ip link del " + std::string(kDummyInterfaceName) + " >> /dev/null 2>&1 || true");
  }

  /**
   * @brief Executes a system command.
   *
   * @param command The command to be executed.
   * @throws std::runtime_error if the command fails to execute.
   */
  void ExecuteSystemCommand(const std::string & command)
  {
    const int res = system(command.c_str());
    if (res != 0) {
      throw std::runtime_error("Failed to execute the command: " + command);
    }
  }

  static constexpr char kDefaultServerIP[] = "21.3.7.147";
  static constexpr char kDefaultServerPort[] = "8080";

protected:
  static constexpr char kDummyInterfaceName[] = "dummy0";
  std::thread server_thread_;
  std::atomic<bool> server_thread_finished_ = false;
};

TEST_F(TestShutdownRemoteHost, RegisterAndCreateTree)
{
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", {}));
}

TEST_F(TestShutdownRemoteHost, MissingServerIPPort)
{
  std::map<std::string, std::string> bb_ports = {
    {"server_port", "1234"},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, MissingServerPortPort)
{
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", "localhost"},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, MissingServerSecretPort)
{
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", "localhost"},
    {"server_port", "1234"},
    {"timeout", "1.0"},
  };
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, MissingTimeoutPort)
{
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", "localhost"},
    {"server_port", "1234"},
    {"secret", "husarion"},
  };
  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, HTTPServerNotAvailable)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "0.1"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, CallShutdownRemoteHost)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "10.0"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  ASSERT_NO_THROW(this->CreateServer(server_ip, server_port));
  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestShutdownRemoteHost, HTTPServerReturnSuccessWithMessage)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  ASSERT_NO_THROW(this->CreateServer(
    server_ip, server_port, "200 OK\\r\\nContent-Type: text/plain\\r\\n\\r\\nSuccess"));
  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestShutdownRemoteHost, HTTPServerReturnFailure)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  ASSERT_NO_THROW(this->CreateServer(
    server_ip, server_port,
    "401 Unauthorized\\r\\nContent-Type: text/plain\\r\\n\\r\\nUnauthorized", 1.0, false));
  auto & tree = GetTree();
  const auto status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(TestShutdownRemoteHost, PingHost)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  ASSERT_NO_THROW(this->CreateServer(server_ip, server_port, "200 OK", 2.0, false));
  auto & tree = GetTree();

  BT::NodeStatus status;
  while (!this->server_thread_finished_) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  }

  ASSERT_NO_THROW(this->StopDummyHost());
  status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(TestShutdownRemoteHost, PingHostTimeout)
{
  const std::string server_ip = kDefaultServerIP;
  const std::string server_port = kDefaultServerPort;
  std::map<std::string, std::string> bb_ports = {
    {"server_ip", server_ip},
    {"server_port", server_port},
    {"secret", "husarion"},
    {"timeout", "1.0"},
  };

  ASSERT_NO_THROW(
    RegisterNodeWithoutParams<husarion_ugv_manager::ShutdownRemoteHost>("ShutdownRemoteHost"));
  ASSERT_NO_THROW(CreateTree("ShutdownRemoteHost", bb_ports));

  ASSERT_NO_THROW(this->CreateServer(server_ip, server_port, "200 OK", 2.0, false));
  auto & tree = GetTree();

  BT::NodeStatus status;
  while (!this->server_thread_finished_) {
    status = tree.tickOnce();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  }

  status = tree.tickWhileRunning(std::chrono::milliseconds(100));
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}
