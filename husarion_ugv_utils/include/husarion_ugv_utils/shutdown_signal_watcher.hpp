// Copyright 2026 Husarion sp. z o.o.
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

#ifndef HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_
#define HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_

#include <pthread.h>

#include <csignal>
#include <ctime>

#include <atomic>
#include <functional>
#include <system_error>
#include <thread>
#include <utility>

namespace husarion_ugv_utils
{

/**
 * @brief Takes SIGINT and SIGTERM away from rclcpp, so a shutdown signal never invalidates the
 * context while an executor is spinning.
 *
 * rclcpp's default handler shuts the context down from another thread, and an executor that is
 * rebuilding its wait set at that moment can throw from places no try/catch around spin()
 * reaches. This watcher blocks both signals in the constructor, so every thread created later
 * inherits the mask, consumes them on its own thread and only calls the given callback. The
 * caller stops spinning and calls rclcpp::shutdown() itself.
 *
 * Construct it as the first object in main(), before rclcpp::init(), and pass
 * rclcpp::SignalHandlerOptions::None to rclcpp::init(). The mask covers only the calling thread
 * and threads created after it: the middleware threads rclcpp::init() starts would otherwise keep
 * both signals unblocked, and a thread started before main() keeps the default action, so a
 * signal routed to such a thread terminates the process without running the callback.
 *
 * The callback runs once, on the first signal. Both signals stay blocked afterwards, so a second
 * Ctrl-C does nothing; a process that hangs in shutdown has to be killed with SIGKILL, which is
 * what launch escalates to. The destructor discards pending signals and restores the old mask; a
 * signal that arrives between the two takes its default action, which only ends a process that is
 * exiting anyway.
 */
class ShutdownSignalWatcher
{
public:
  ShutdownSignalWatcher()
  {
    sigemptyset(&signals_);
    sigaddset(&signals_, SIGINT);
    sigaddset(&signals_, SIGTERM);
    // Without the mask rclcpp would have no signal handling at all, since the caller passes
    // SignalHandlerOptions::None.
    const int ret = pthread_sigmask(SIG_BLOCK, &signals_, &old_mask_);
    if (ret != 0) {
      throw std::system_error(ret, std::generic_category(), "pthread_sigmask");
    }
  }

  /**
   * @brief Starts the watcher thread. on_signal runs on that thread on the first SIGINT or
   * SIGTERM.
   */
  void Start(std::function<void()> on_signal)
  {
    if (watcher_.joinable()) {
      return;
    }
    watcher_ = std::thread([this, on_signal = std::move(on_signal)]() {
      // A timeout instead of a bare sigwait, so the destructor can end the thread when the process
      // exits without a signal.
      const timespec poll_interval{0, 100 * 1000 * 1000};
      while (!done_.load()) {
        const int sig = sigtimedwait(&signals_, nullptr, &poll_interval);
        if (sig == SIGINT || sig == SIGTERM) {
          on_signal();
          return;
        }
      }
    });
  }

  ~ShutdownSignalWatcher()
  {
    done_.store(true);
    if (watcher_.joinable()) {
      watcher_.join();
    }
    // A second signal still pending would otherwise take its default action and kill the process.
    const timespec no_wait{0, 0};
    while (sigtimedwait(&signals_, nullptr, &no_wait) > 0) {
    }
    pthread_sigmask(SIG_SETMASK, &old_mask_, nullptr);
  }

  ShutdownSignalWatcher(const ShutdownSignalWatcher &) = delete;
  ShutdownSignalWatcher & operator=(const ShutdownSignalWatcher &) = delete;

private:
  sigset_t signals_;
  sigset_t old_mask_;
  std::atomic<bool> done_{false};
  std::thread watcher_;
};

}  // namespace husarion_ugv_utils

#endif  // HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_
