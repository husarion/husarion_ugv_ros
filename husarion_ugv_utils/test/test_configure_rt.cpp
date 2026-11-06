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

#include <pthread.h>
#include <sched.h>

#include <stdexcept>
#include <thread>

#include <gtest/gtest.h>

#include "husarion_ugv_utils/configure_rt.hpp"

namespace
{

cpu_set_t CurrentAffinity()
{
  cpu_set_t cpu_set;
  CPU_ZERO(&cpu_set);
  pthread_getaffinity_np(pthread_self(), sizeof(cpu_set), &cpu_set);
  return cpu_set;
}

// The test machine may have no RT kernel or no permission for SCHED_FIFO, so the priority half
// may throw. The pinning half must have happened either way.
void ConfigureRTIgnoringPriorityErrors(const unsigned priority, const int cpu)
{
  try {
    husarion_ugv_utils::ConfigureRT(priority, cpu);
  } catch (const std::runtime_error &) {
  }
}

}  // namespace

TEST(TestConfigureRT, PinsToTheRequestedCPU)
{
  std::thread([]() {
    ConfigureRTIgnoringPriorityErrors(10, 0);
    const auto cpu_set = CurrentAffinity();
    EXPECT_EQ(CPU_COUNT(&cpu_set), 1);
    EXPECT_TRUE(CPU_ISSET(0, &cpu_set));
  }).join();
}

TEST(TestConfigureRT, NegativeCPULeavesAffinityAlone)
{
  std::thread([]() {
    const auto before = CurrentAffinity();
    ConfigureRTIgnoringPriorityErrors(10, -1);
    auto after = CurrentAffinity();
    EXPECT_TRUE(CPU_EQUAL(&before, &after));
  }).join();
}

TEST(TestConfigureRT, ImpossibleCPUIsNotFatal)
{
  std::thread([]() {
    const auto before = CurrentAffinity();
    ConfigureRTIgnoringPriorityErrors(10, CPU_SETSIZE + 5);
    auto after = CurrentAffinity();
    EXPECT_TRUE(CPU_EQUAL(&before, &after));
  }).join();
}

TEST(TestConfigureRT, InvalidPriorityStillThrows)
{
  EXPECT_THROW(husarion_ugv_utils::ConfigureRT(100, -1), std::runtime_error);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
