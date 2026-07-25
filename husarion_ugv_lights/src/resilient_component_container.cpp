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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

// Drop-in replacement for rclcpp_components' component_container that
// tolerates losing the shutdown race. On SIGTERM the signal handler
// invalidates the context while spin() is mid wait-set rebuild -
// rcl_wait_set_init then fails with "context is not valid", the RCLError
// escapes spin() and the stock container aborts on every container stop.
// Upstream rclcpp executor race; a post-shutdown RCLError is benign, so
// swallow exactly that and exit clean. Anything thrown while the context
// is still alive is a real error and stays fatal.
int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  try {
    exec->spin();
  } catch (const rclcpp::exceptions::RCLError & e) {
    if (rclcpp::ok()) {
      throw;
    }
  }
  rclcpp::shutdown();
  return 0;
}
