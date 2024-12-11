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

#include <filesystem>
#include <fstream>
#include <memory>
#include <string>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "husarion_ugv_diagnostics/filesystem.hpp"
#include "husarion_ugv_diagnostics/system_monitor_node.hpp"

class MockFilesystem : public husarion_ugv_diagnostics::FilesystemInterface
{
public:
  MOCK_METHOD(
    uintmax_t, GetSpaceCapacity, (const std::string & filesystem_path), (const, override));

  MOCK_METHOD(
    uintmax_t, GetSpaceAvailable, (const std::string & filesystem_path), (const, override));

  MOCK_METHOD(std::string, ReadFile, (const std::string & file_path), (const, override));
};

class SystemMonitorNodeWrapper : public husarion_ugv_diagnostics::SystemMonitorNode
{
public:
  SystemMonitorNodeWrapper(std::shared_ptr<MockFilesystem> filesystem)
  : husarion_ugv_diagnostics::SystemMonitorNode("test_system_statics", filesystem)
  {
  }

  std::vector<float> GetCoresUsages() const
  {
    return husarion_ugv_diagnostics::SystemMonitorNode::GetCoresUsages();
  }

  float GetCPUMeanUsage(const std::vector<float> & usages) const
  {
    return husarion_ugv_diagnostics::SystemMonitorNode::GetCPUMeanUsage(usages);
  }

  float GetCPUTemperature() const
  {
    return husarion_ugv_diagnostics::SystemMonitorNode::GetCPUTemperature();
  }

  float GetRAMUsage() const { return husarion_ugv_diagnostics::SystemMonitorNode::GetRAMUsage(); }

  float GetDiskUsage() const { return husarion_ugv_diagnostics::SystemMonitorNode::GetDiskUsage(); }

  husarion_ugv_msgs::msg::SystemStatus SystemStatusToMessage(
    const husarion_ugv_diagnostics::SystemStatus & status)

  {
    return husarion_ugv_diagnostics::SystemMonitorNode::SystemStatusToMessage(status);
  }
};

class TestSystemMonitorNode : public testing::Test
{
public:
  TestSystemMonitorNode();

protected:
  std::shared_ptr<MockFilesystem> filesystem_;
  std::unique_ptr<SystemMonitorNodeWrapper> system_monitor_;
};

TestSystemMonitorNode::TestSystemMonitorNode()
{
  filesystem_ = std::make_unique<MockFilesystem>();
  system_monitor_ = std::make_unique<SystemMonitorNodeWrapper>(filesystem_);
}

TEST_F(TestSystemMonitorNode, GetCPUMeanUsageCorrectInput)
{
  const std::vector<float> test_usages = {100.0, 0.0, 45.0, 55.0};
  const auto expected_mean_usage = 50.0;

  const auto mean_usage = system_monitor_->GetCPUMeanUsage(test_usages);
  EXPECT_FLOAT_EQ(expected_mean_usage, mean_usage);
}

TEST_F(TestSystemMonitorNode, GetCPUMeanUsageEmptyInput)
{
  const std::vector<float> test_usages = {};

  const auto mean_usage = system_monitor_->GetCPUMeanUsage(test_usages);

  EXPECT_TRUE(std::isnan(mean_usage));
}

TEST_F(TestSystemMonitorNode, GetCPUMeanUsageOverloaded)
{
  const std::vector<float> test_usages = {150.0, 50.0, 50.0, 50.0};

  EXPECT_THROW(system_monitor_->GetCPUMeanUsage(test_usages), std::invalid_argument);
}

TEST_F(TestSystemMonitorNode, GetCPUMeanUsageUnderloaded)
{
  const std::vector<float> test_usages = {-50.0, 50.0, 50.0, 50.0};

  EXPECT_THROW(system_monitor_->GetCPUMeanUsage(test_usages), std::invalid_argument);
}

TEST_F(TestSystemMonitorNode, GetCPUTemperatureValidFile)
{
  ON_CALL(*filesystem_, ReadFile(testing::_)).WillByDefault(testing::Return("25000"));

  const auto temperature = system_monitor_->GetCPUTemperature();

  EXPECT_EQ(25.0, temperature);
}

TEST_F(TestSystemMonitorNode, GetCPUTemperatureMissingFile)
{
  ON_CALL(*filesystem_, ReadFile(testing::_))
    .WillByDefault(testing::Throw(std::invalid_argument("File not found")));

  const auto temperature = system_monitor_->GetCPUTemperature();

  EXPECT_TRUE(std::isnan(temperature));
}

TEST_F(TestSystemMonitorNode, GetDiskUsageValidFilesystem)
{
  ON_CALL(*filesystem_, GetSpaceCapacity(testing::_)).WillByDefault(testing::Return(100000));
  ON_CALL(*filesystem_, GetSpaceAvailable(testing::_)).WillByDefault(testing::Return(50000));

  const auto disk_usage = system_monitor_->GetDiskUsage();

  EXPECT_EQ(50.0, disk_usage);
}

TEST_F(TestSystemMonitorNode, GetDiskUsageInvalidFilesystem)
{
  ON_CALL(*filesystem_, GetSpaceCapacity(testing::_)).WillByDefault(testing::Return(100000));
  ON_CALL(*filesystem_, GetSpaceAvailable(testing::_))
    .WillByDefault(testing::Throw(std::invalid_argument{"Filesystem not found"}));

  const auto disk_usage = system_monitor_->GetDiskUsage();

  EXPECT_TRUE(std::isnan(disk_usage));
}

TEST_F(TestSystemMonitorNode, SystemStatusToMessage)
{
  husarion_ugv_diagnostics::SystemStatus test_status;
  test_status.core_usages = {50.0, 50.0, 50.0};
  test_status.cpu_mean_usage = 50.0;
  test_status.cpu_temperature = 36.6;
  test_status.ram_usage = 30.0;
  test_status.disk_usage = 60.0;

  const auto message = system_monitor_->SystemStatusToMessage(test_status);

  EXPECT_EQ(test_status.core_usages, message.cpu_percent);
  EXPECT_FLOAT_EQ(test_status.cpu_mean_usage, message.avg_load_percent);
  EXPECT_FLOAT_EQ(test_status.cpu_temperature, message.cpu_temp);
  EXPECT_FLOAT_EQ(test_status.ram_usage, message.ram_usage_percent);
  EXPECT_FLOAT_EQ(test_status.disk_usage, message.disc_usage_percent);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(0, nullptr);

  auto result = RUN_ALL_TESTS();

  rclcpp::shutdown();
  return result;
}
