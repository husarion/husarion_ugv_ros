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

#include "husarion_ugv_utils/shutdown_signal_watcher.hpp"

// Drop-in replacement for rclcpp_components' component_container that survives
// container stops. The stock container lets rclcpp's signal handler invalidate
// the context while spin() is mid wait-set rebuild; the resulting RCLError can
// escape through paths no try/catch around spin() reaches (a second throw
// during unwinding calls terminate before any handler runs - catching and
// swallowing proved insufficient on the bench). Instead the shutdown signals
// never touch the context at all: a watcher thread consumes SIGINT/SIGTERM and
// only cancels the executor, spin() returns normally with the context still
// valid, and shutdown happens after - so there is nothing mid-flight to throw.
int main(int argc, char * argv[])
{
  husarion_ugv_utils::ShutdownSignalWatcher signal_watcher;
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto exec = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  auto node = std::make_shared<rclcpp_components::ComponentManager>(exec);
  exec->add_node(node);
  signal_watcher.Start([exec]() { exec->cancel(); });
  exec->spin();
  // Nodes must outlive shutdown(): rclcpp::on_shutdown hooks registered by
  // loaded components capture their node and run inside this call.
  try {
    rclcpp::shutdown();
  } catch (const rclcpp::exceptions::RCLError &) {
    // A loaded component's on_shutdown hook touched the ROS graph after the
    // context died (the lights driver's LED-control release used to via
    // wait_for_service) - too late for anything to act on it; exit clean
    // instead of aborting.
  }
  return 0;
}
