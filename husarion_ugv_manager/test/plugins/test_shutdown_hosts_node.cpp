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

#include <memory>
#include <string>
#include <vector>

#include "gmock/gmock.h"
#include "gtest/gtest.h"

#include "husarion_ugv_manager/plugins/shutdown_hosts_node.hpp"

class MockShutdownHost : public husarion_ugv_manager::ShutdownHostInterface
{
public:
  MockShutdownHost(std::size_t hash) : husarion_ugv_manager::ShutdownHostInterface(hash) {}
  ~MockShutdownHost() {}

  MOCK_METHOD(void, Call, (), (override));
  MOCK_METHOD(void, Halt, (), (override));
  MOCK_METHOD(std::string, GetIp, (), (const, override));
  MOCK_METHOD(std::string, GetError, (), (const, override));
  MOCK_METHOD(std::string, GetOutput, (), (const, override));
  MOCK_METHOD(husarion_ugv_manager::ShutdownHostState, GetState, (), (const, override));

  using NiceMock = testing::NiceMock<MockShutdownHost>;
};

class ShutdownHostsWrapper : public husarion_ugv_manager::ShutdownHosts
{
public:
  ShutdownHostsWrapper(const std::string & name, const BT::NodeConfig & conf)
  : husarion_ugv_manager::ShutdownHosts(name, conf)
  {
  }

  void RemoveDuplicatedHosts(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> & hosts);
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> & GetHosts();

  BT::NodeStatus onStart();
  BT::NodeStatus onRunning();

  virtual bool UpdateHosts(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> & hosts)
    override final;
  void SetHostsAndSuccess(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts,
    const bool returned_status);

  static BT::PortsList providedPorts() { return {}; }

private:
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts_to_set;
  bool update_hosts_success_ = true;
};

void ShutdownHostsWrapper::RemoveDuplicatedHosts(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> & hosts)
{
  husarion_ugv_manager::ShutdownHosts::RemoveDuplicatedHosts(hosts);
}

std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> &
ShutdownHostsWrapper::GetHosts()
{
  return hosts_;
}

BT::NodeStatus ShutdownHostsWrapper::onRunning()
{
  return husarion_ugv_manager::ShutdownHosts::onRunning();
}

BT::NodeStatus ShutdownHostsWrapper::onStart()
{
  return husarion_ugv_manager::ShutdownHosts::onStart();
}

bool ShutdownHostsWrapper::UpdateHosts(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> & hosts)
{
  hosts = hosts_to_set;
  return update_hosts_success_;
}

void ShutdownHostsWrapper::SetHostsAndSuccess(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts,
  const bool returned_status)
{
  hosts_to_set = hosts;
  update_hosts_success_ = returned_status;
}

class ShutdownHostsNodeTest : public testing::Test
{
public:
  void CreateWrapper(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts,
    const bool success);

protected:
  std::unique_ptr<ShutdownHostsWrapper> wrapper;
};

void ShutdownHostsNodeTest::CreateWrapper(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts,
  const bool success)
{
  BT::NodeConfig conf;
  wrapper = std::make_unique<ShutdownHostsWrapper>("Duplicated hosts", conf);
  wrapper->SetHostsAndSuccess(hosts, success);
}

TEST_F(ShutdownHostsNodeTest, GoodRemovingDuplicatedHosts)
{
  CreateWrapper(
    {std::make_shared<husarion_ugv_manager::ShutdownHost>("127.0.0.1", "3003", "password", 1.0),
     std::make_shared<husarion_ugv_manager::ShutdownHost>("localhost", "3003", "password", 1.0),
     std::make_shared<husarion_ugv_manager::ShutdownHost>("localhost", "3003", "password", 1.0),
     std::make_shared<husarion_ugv_manager::ShutdownHost>("localhost", "3003", "password", 1.0),
     std::make_shared<husarion_ugv_manager::ShutdownHost>("127.0.0.1", "3003", "password", 1.0),
     std::make_shared<husarion_ugv_manager::ShutdownHost>("127.0.0.1", "8080", "password", 1.0)},
    true);
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHostInterface>> hosts;
  ASSERT_TRUE(wrapper->UpdateHosts(hosts));
  ASSERT_EQ(hosts.size(), 6);
  wrapper->RemoveDuplicatedHosts(hosts);
  ASSERT_EQ(hosts.size(), 3);
}

TEST_F(ShutdownHostsNodeTest, SuccessMultipleHosts)
{
  auto host_1 = std::make_shared<MockShutdownHost::NiceMock>(0);
  auto host_2 = std::make_shared<MockShutdownHost::NiceMock>(1);
  CreateWrapper({host_1, host_2}, true);

  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  EXPECT_CALL(*host_1, Call()).Times(1);
  EXPECT_CALL(*host_2, Call()).Times(1);
  EXPECT_CALL(*host_1, GetState())
    .WillOnce(testing::Return(husarion_ugv_manager::ShutdownHostState::SUCCESS));
  EXPECT_CALL(*host_2, GetState())
    .WillOnce(testing::Return(husarion_ugv_manager::ShutdownHostState::SKIPPED));

  while (status == BT::NodeStatus::RUNNING) {
    status = wrapper->onRunning();
  }
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsNodeTest, ResponseReceived)
{
  auto host = std::make_shared<MockShutdownHost::NiceMock>(0);
  CreateWrapper({host}, true);

  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  EXPECT_CALL(*host, Call()).Times(1);
  EXPECT_CALL(*host, GetState())
    .WillOnce(testing::Return(husarion_ugv_manager::ShutdownHostState::RESPONSE_RECEIVED));
  EXPECT_CALL(*host, GetIp()).WillOnce(testing::Return("0.0.0.0"));
  EXPECT_CALL(*host, GetOutput()).WillOnce(testing::Return("Success"));

  status = wrapper->onRunning();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  EXPECT_CALL(*host, Call()).Times(1);
  EXPECT_CALL(*host, GetState())
    .WillOnce(testing::Return(husarion_ugv_manager::ShutdownHostState::SUCCESS));
  EXPECT_CALL(*host, GetIp()).WillOnce(testing::Return("0.0.0.0"));

  // call twice first to get host state, then to check hosts list and return value
  status = wrapper->onRunning();
  status = wrapper->onRunning();
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsNodeTest, HostStateFailure)
{
  auto host = std::make_shared<MockShutdownHost::NiceMock>(0);
  CreateWrapper({host}, true);

  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);

  EXPECT_CALL(*host, Call()).Times(1);
  EXPECT_CALL(*host, GetState())
    .WillOnce(testing::Return(husarion_ugv_manager::ShutdownHostState::FAILURE));

  while (status == BT::NodeStatus::RUNNING) {
    status = wrapper->onRunning();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(wrapper->GetFailedHosts().size(), 1);
}

TEST_F(ShutdownHostsNodeTest, FailedWhenUpdateHostReturnsFalse)
{
  CreateWrapper(
    {std::make_shared<husarion_ugv_manager::ShutdownHost>("127.0.0.1", "3003", "password", 1.0)},
    false);

  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST_F(ShutdownHostsNodeTest, FailedWhenHostsAreEmpty)
{
  CreateWrapper({}, true);

  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
