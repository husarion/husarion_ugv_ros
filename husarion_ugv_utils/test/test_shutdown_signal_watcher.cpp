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

#include <pthread.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <ctime>
#include <thread>

#include "gtest/gtest.h"

#include "husarion_ugv_utils/shutdown_signal_watcher.hpp"

TEST(TestShutdownSignalWatcher, CallbackRunsOnceOnSigterm)
{
  sigset_t mask_before;
  ASSERT_EQ(0, pthread_sigmask(SIG_BLOCK, nullptr, &mask_before));

  std::atomic<int> calls{0};
  {
    husarion_ugv_utils::ShutdownSignalWatcher watcher;
    watcher.Start([&calls]() { calls++; });

    // Process-directed, so the watcher thread is the one that consumes it.
    ASSERT_EQ(0, kill(getpid(), SIGTERM));
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (calls.load() == 0 && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_EQ(1, calls.load());

    // The second signal stays pending and blocked, the callback does not run again.
    ASSERT_EQ(0, kill(getpid(), SIGTERM));
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
  }
  EXPECT_EQ(1, calls.load());

  // The destructor discarded the second signal and restored the mask it found.
  sigset_t pending;
  ASSERT_EQ(0, sigpending(&pending));
  EXPECT_FALSE(sigismember(&pending, SIGTERM));
  sigset_t mask;
  ASSERT_EQ(0, pthread_sigmask(SIG_BLOCK, nullptr, &mask));
  EXPECT_EQ(sigismember(&mask_before, SIGTERM), sigismember(&mask, SIGTERM));
  EXPECT_EQ(sigismember(&mask_before, SIGINT), sigismember(&mask, SIGINT));
}

TEST(TestShutdownSignalWatcher, SecondStartIsIgnored)
{
  husarion_ugv_utils::ShutdownSignalWatcher watcher;
  watcher.Start([]() {});
  watcher.Start([]() {});
}

TEST(TestShutdownSignalWatcher, DestructorEndsThreadWithoutSignal)
{
  const auto start = std::chrono::steady_clock::now();
  {
    husarion_ugv_utils::ShutdownSignalWatcher watcher;
    watcher.Start([]() {});
  }
  EXPECT_LT(std::chrono::steady_clock::now() - start, std::chrono::seconds(1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
