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

#ifndef HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_
#define HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_

#include <algorithm>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/basic_types.h"
#include "yaml-cpp/yaml.h"

#include "husarion_ugv_manager/plugins/shutdown_host.hpp"
#include "husarion_ugv_manager/plugins/shutdown_hosts_node.hpp"
#include "husarion_ugv_utils/yaml_utils.hpp"

namespace husarion_ugv_manager
{

class ShutdownHostsFromFile : public ShutdownHosts
{
public:
  ShutdownHostsFromFile(const std::string & name, const BT::NodeConfig & conf)
  : ShutdownHosts(name, conf)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::string>(
        "shutdown_hosts_file", "Absolute path to a YAML file listing the hosts to shut down."),
      BT::InputPort<float>(
        "timeout_override",
        "Optional. Caps each host's timeout from the file at this many seconds. Unset, zero or "
        "negative leaves the timeouts from the file."),
    };
  }

  /**
   * @brief The timeout a host gets: its own, capped by the override when one is set.
   */
  static float EffectiveTimeout(const float host_timeout, const std::optional<float> & override_s)
  {
    if (!override_s.has_value() || !(override_s.value() > 0.0f)) {
      return host_timeout;
    }
    return std::min(host_timeout, override_s.value());
  }

private:
  bool UpdateHosts(std::vector<std::shared_ptr<ShutdownHostInterface>> & hosts) override;
};

}  // namespace husarion_ugv_manager

#endif  // HUSARION_UGV_MANAGER_HUSARION_UGV_MANAGER_PLUGINS_ACTION_SHUTDOWN_HOSTS_FROM_FILE_NODE_HPP_
