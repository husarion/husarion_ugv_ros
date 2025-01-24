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

#include "husarion_ugv_manager/plugins/shutdown_hosts_node.hpp"

class ShutdownHostsNodeWrapper : public husarion_ugv_manager::ShutdownHosts
{
public:
  ShutdownHostsNodeWrapper(const std::string & name, const BT::NodeConfig & conf)
  : husarion_ugv_manager::ShutdownHosts(name, conf)
  {
  }
  void RemoveDuplicatedHosts(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> & hosts);
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> & GetHosts();
  BT::NodeStatus onRunning();
  BT::NodeStatus onStart();

  virtual bool UpdateHosts(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> & hosts) override final;
  void SetHostsAndSuccess(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts,
    const bool returned_status);

  static BT::PortsList providedPorts()
  {
    return {

    };
  }

private:
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts_to_set;
  bool update_hosts_success_ = true;
};

void ShutdownHostsNodeWrapper::RemoveDuplicatedHosts(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> & hosts)
{
  husarion_ugv_manager::ShutdownHosts::RemoveDuplicatedHosts(hosts);
}

std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> &
ShutdownHostsNodeWrapper::GetHosts()
{
  return hosts_;
}

BT::NodeStatus ShutdownHostsNodeWrapper::onRunning()
{
  return husarion_ugv_manager::ShutdownHosts::onRunning();
}

BT::NodeStatus ShutdownHostsNodeWrapper::onStart()
{
  return husarion_ugv_manager::ShutdownHosts::onStart();
}

bool ShutdownHostsNodeWrapper::UpdateHosts(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> & hosts)
{
  hosts = hosts_to_set;
  return update_hosts_success_;
}

void ShutdownHostsNodeWrapper::SetHostsAndSuccess(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts,
  const bool returned_status)
{
  hosts_to_set = hosts;
  update_hosts_success_ = returned_status;
}

class ShutdownHostsNodeTest : public testing::Test
{
public:
  void CreateWrapper(
    std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts, const bool success);

protected:
  std::unique_ptr<ShutdownHostsNodeWrapper> wrapper;
};

void ShutdownHostsNodeTest::CreateWrapper(
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts, const bool success)
{
  BT::NodeConfig conf;
  wrapper = std::make_unique<ShutdownHostsNodeWrapper>("Duplicated hosts", conf);
  wrapper->SetHostsAndSuccess(hosts, success);
}

TEST_F(ShutdownHostsNodeTest, GoodRemovingDuplicatedHosts)
{
  CreateWrapper(
    {std::make_shared<husarion_ugv_manager::ShutdownHost>(
       "127.0.0.1", "husarion", 22, "echo HelloWorld", 1.0, true),
     std::make_shared<husarion_ugv_manager::ShutdownHost>(
       "localhost", "husarion", 22, "echo HelloWorld", 1.0, true),
     std::make_shared<husarion_ugv_manager::ShutdownHost>(
       "localhost", "husarion", 22, "echo HelloWorld", 1.0, true)},
    true);
  std::vector<std::shared_ptr<husarion_ugv_manager::ShutdownHost>> hosts;
  ASSERT_TRUE(wrapper->UpdateHosts(hosts));
  ASSERT_EQ(hosts.size(), 3);
  wrapper->RemoveDuplicatedHosts(hosts);
  ASSERT_EQ(hosts.size(), 2);
}

TEST_F(ShutdownHostsNodeTest, FailedWhenUpdateHostReturnsFalse)
{
  CreateWrapper(
    {std::make_shared<husarion_ugv_manager::ShutdownHost>(
      "127.0.0.1", "husarion", 22, "echo HelloWorld", 1.0, true)},
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

TEST_F(ShutdownHostsNodeTest, CheckFailedHosts)
{
  CreateWrapper(
    {std::make_shared<husarion_ugv_manager::ShutdownHost>(
      "127.0.0.1", "husarion", 22, "echo HelloWorld", 1.0, true)},
    true);
  auto status = wrapper->onStart();
  EXPECT_EQ(status, BT::NodeStatus::RUNNING);
  while (status == BT::NodeStatus::RUNNING) {
    status = wrapper->onRunning();
  }
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
  EXPECT_EQ(wrapper->GetFailedHosts().size(), 1);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();

  return result;
}
