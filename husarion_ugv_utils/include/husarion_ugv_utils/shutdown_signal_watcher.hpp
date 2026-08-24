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

#ifndef HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_
#define HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_

#include <csignal>
#include <ctime>

#include <atomic>
#include <functional>
#include <thread>
#include <utility>

namespace husarion_ugv_utils
{

/**
 * @brief Takes SIGINT/SIGTERM away from rclcpp so shutdown cannot invalidate the context
 * mid-spin.
 *
 * rclcpp's default signal handler tears the context down on a separate thread while an
 * executor may be mid wait-set rebuild; the resulting RCLError can escape spin() through
 * paths no try/catch reaches and terminate the process on a routine container stop. This
 * watcher blocks the shutdown signals before any thread exists (every later thread inherits
 * the mask), consumes them on a dedicated thread and only invokes a caller-provided
 * callback - typically Executor::cancel() or a loop-exit flag. The context stays valid
 * until the caller decides spinning is over and calls rclcpp::shutdown() itself.
 *
 * Construct as the first object in main(), before rclcpp::init(), and pass
 * rclcpp::SignalHandlerOptions::None to init so the default handler never competes.
 */
class ShutdownSignalWatcher
{
public:
  ShutdownSignalWatcher()
  {
    sigemptyset(&signals_);
    sigaddset(&signals_, SIGINT);
    sigaddset(&signals_, SIGTERM);
    pthread_sigmask(SIG_BLOCK, &signals_, nullptr);
  }

  /**
   * @brief Starts the watcher thread; on_signal runs on that thread once, on the first
   * SIGINT/SIGTERM.
   */
  void Start(std::function<void()> on_signal)
  {
    watcher_ = std::thread([this, on_signal = std::move(on_signal)]() {
      // Poll with a timeout instead of a bare sigwait so the destructor can end the
      // thread when the process exits without ever receiving a signal.
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
  }

  ShutdownSignalWatcher(const ShutdownSignalWatcher &) = delete;
  ShutdownSignalWatcher & operator=(const ShutdownSignalWatcher &) = delete;

private:
  sigset_t signals_;
  std::atomic<bool> done_{false};
  std::thread watcher_;
};

}  // namespace husarion_ugv_utils

#endif  // HUSARION_UGV_UTILS_HUSARION_UGV_UTILS_SHUTDOWN_SIGNAL_WATCHER_HPP_
