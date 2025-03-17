// Copyright 2025 Husarion sp. z o.o.
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

#include "gtest/gtest.h"

#include "husarion_ugv_utils/networking_utils.hpp"

class PortBinder : public ::testing::Test
{
public:
  int OpenPort(int port)
  {
    sock_ = socket(AF_INET, SOCK_STREAM, 0);
    if (sock_ == -1) {
      return -1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr));
    return sock_;
  }

  ~PortBinder() { close(sock_); }

protected:
  int sock_;
};

TEST_F(PortBinder, TestClosedPort)
{
  EXPECT_GT(OpenPort(1667), -1);
  EXPECT_FALSE(husarion_ugv_utils::IsPortAvailable(1667));
}

TEST_F(PortBinder, TestOpenPort) { EXPECT_TRUE(husarion_ugv_utils::IsPortAvailable(1668)); }

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
