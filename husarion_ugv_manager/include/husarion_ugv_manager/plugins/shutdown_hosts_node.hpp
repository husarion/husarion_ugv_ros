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

#ifndef HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_
#define HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_

#include <algorithm>
#include <memory>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/basic_types.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"

#include "husarion_ugv_manager/plugins/shutdown_host.hpp"

#include "husarion_ugv_manager/behavior_tree_utils.hpp"

namespace husarion_ugv_manager
{

class ShutdownHosts : public BT::StatefulActionNode
{
public:
  explicit ShutdownHosts(const std::string & name, const BT::NodeConfig & conf)
  : BT::StatefulActionNode(name, conf)
  {
    this->logger_ = std::make_shared<rclcpp::Logger>(rclcpp::get_logger(name));
  }

  virtual ~ShutdownHosts() = default;

  // method to be implemented by user
  virtual bool UpdateHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts) = 0;

  // method that can be overridden by user
  virtual BT::NodeStatus PostProcess()
  {
    // return success only when all hosts succeeded
    if (this->failed_hosts_.size() == 0) {
      return BT::NodeStatus::SUCCESS;
    }
    return BT::NodeStatus::FAILURE;
  }

  std::vector<std::size_t> const GetFailedHosts() { return this->failed_hosts_; }

protected:
  BT::NodeStatus onStart()
  {
    if (!UpdateHosts(this->hosts_)) {
      RCLCPP_ERROR_STREAM(*this->logger_, GetLoggerPrefix(name()) << "Cannot update hosts!");
      return BT::NodeStatus::FAILURE;
    }

    RemoveDuplicatedHosts(this->hosts_);
    if (this->hosts_.size() <= 0) {
      RCLCPP_ERROR_STREAM(
        *this->logger_, GetLoggerPrefix(name()) << "Hosts list is empty! Check configuration!");
      return BT::NodeStatus::FAILURE;
    }
    this->hosts_to_check_.resize(this->hosts_.size());
    std::iota(this->hosts_to_check_.begin(), this->hosts_to_check_.end(), 0);
    return BT::NodeStatus::RUNNING;
  }

  BT::NodeStatus onRunning()
  {
    if (this->hosts_to_check_.size() <= 0) {
      return PostProcess();
    }

    if (this->check_host_index_ >= this->hosts_to_check_.size()) {
      this->check_host_index_ = 0;
    }

    auto host_index = this->hosts_to_check_[this->check_host_index_];
    auto host = this->hosts_[host_index];
    host->Call();

    switch (host->GetState()) {
      case ShutdownHostState::RESPONSE_RECEIVED:
        RCLCPP_INFO_STREAM(
          *this->logger_, GetLoggerPrefix(name())
                            << "Device at: " << host->GetIp() << " response:\n"
                            << host->GetOutput());

        check_host_index_++;
        break;

      case ShutdownHostState::SUCCESS:
        RCLCPP_INFO_STREAM(
          *this->logger_, GetLoggerPrefix(name())
                            << "Successfully shutdown device at: " << host->GetIp());
        this->succeeded_hosts_.push_back(host_index);
        this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
        break;

      case ShutdownHostState::FAILURE:
        RCLCPP_WARN_STREAM(
          *this->logger_, GetLoggerPrefix(name())
                            << "Failed to shutdown device at: " << host->GetIp()
                            << "\nError: " << host->GetError());

        this->failed_hosts_.push_back(host_index);
        this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
        break;

      case ShutdownHostState::SKIPPED:
        RCLCPP_WARN_STREAM(
          *this->logger_, GetLoggerPrefix(name())
                            << "Device at: " << host->GetIp() << " not available, skipping...");

        this->skipped_hosts_.push_back(host_index);
        this->hosts_to_check_.erase(this->hosts_to_check_.begin() + this->check_host_index_);
        break;

      default:
        this->check_host_index_++;
        break;
    }

    return BT::NodeStatus::RUNNING;
  }

  void RemoveDuplicatedHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts)
  {
    auto comp = [](
                  const std::shared_ptr<ShutdownHostInterface> & lhs,
                  const std::shared_ptr<ShutdownHostInterface> & rhs) { return *lhs < *rhs; };

    std::set<std::shared_ptr<ShutdownHostInterface>, decltype(comp)> seen(comp);

    hosts.erase(
      std::remove_if(
        hosts.begin(), hosts.end(),
        [&](const std::shared_ptr<ShutdownHostInterface> & host) {
          if (!seen.count(host)) {
            seen.insert(host);
            return false;
          } else {
            RCLCPP_WARN_STREAM(
              *this->logger_, GetLoggerPrefix(name()) << "Found duplicate host: " << host->GetIp()
                                                      << " Processing only the first occurrence.");
            return true;
          }
        }),
      hosts.end());
  }

  void onHalted()
  {
    for (auto & host : this->hosts_) {
      host->Halt();
    }
  }

  std::shared_ptr<rclcpp::Logger> logger_;
  std::size_t check_host_index_ = 0;
  std::vector<std::shared_ptr<ShutdownHostInterface>> hosts_;
  std::vector<std::size_t> hosts_to_check_;
  std::vector<std::size_t> skipped_hosts_;
  std::vector<std::size_t> succeeded_hosts_;
  std::vector<std::size_t> failed_hosts_;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_PLUGINS_SHUTDOWN_HOSTS_NODE_HPP_
