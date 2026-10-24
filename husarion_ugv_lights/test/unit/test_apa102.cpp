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

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <memory>
#include <vector>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "husarion_ugv_lights/apa102.hpp"

static constexpr char kMockDeviceName[] = "/dev/mocked_device";
static constexpr int kStartFrame = 0x00;
static constexpr int kEndFrame = 0xFF;

class MockSPIDevice : public husarion_ugv_lights::SPIDeviceInterface
{
public:
  MOCK_METHOD(int, Open, (const std::string & device), (override));
  MOCK_METHOD(int, IOControl, (int fd, unsigned long request, const void * arg), (override));
  MOCK_METHOD(int, Close, (int fd), (override));

  // Nice mock suppresses warnings about uninteresting calls
  using NiceMock = testing::NiceMock<MockSPIDevice>;
};

class APA102Wrapper : public husarion_ugv_lights::APA102
{
public:
  APA102Wrapper(
    std::shared_ptr<MockSPIDevice> spi_device, const std::string & device_name,
    const std::uint32_t speed = 800000, const std::size_t max_transfer_bytes = 0)
  : APA102(spi_device, device_name, speed, false, max_transfer_bytes)
  {
  }

  std::vector<std::uint8_t> RGBAFrameToBGRBuffer(const std::vector<std::uint8_t> & frame) const
  {
    return APA102::RGBAFrameToBGRBuffer(frame);
  }
  std::uint16_t GetGlobalBrightness() const { return global_brightness_; }
};

class TestAPA102 : public testing::Test
{
protected:
  TestAPA102();
  ~TestAPA102() { apa102_.reset(); }

  std::shared_ptr<MockSPIDevice> spi_device_;
  std::unique_ptr<APA102Wrapper> apa102_;
};

TestAPA102::TestAPA102()
{
  spi_device_ = std::make_shared<MockSPIDevice::NiceMock>();

  apa102_ = std::make_unique<APA102Wrapper>(spi_device_, kMockDeviceName);
}

TEST(TestInitialization, InvalidDevices)
{
  auto spi_device = std::make_shared<MockSPIDevice::NiceMock>();

  // Return -1 to simulate failed device opening
  ON_CALL(*spi_device, Open(kMockDeviceName)).WillByDefault(testing::Return(-1));

  EXPECT_THROW(APA102Wrapper(spi_device, kMockDeviceName), std::ios_base::failure);
}

TEST(TestInitialization, SetModeFailure)
{
  auto spi_device = std::make_shared<MockSPIDevice::NiceMock>();

  // Return -1 to simulate failed setting mode
  ON_CALL(*spi_device, IOControl(testing::_, SPI_IOC_WR_MODE32, testing::_))
    .WillByDefault(testing::Return(-1));
  EXPECT_CALL(*spi_device, Close(testing::_)).Times(1);

  EXPECT_THROW(APA102Wrapper(spi_device, kMockDeviceName), std::ios_base::failure);
}

TEST(TestInitialization, SetBitsFailure)
{
  auto spi_device = std::make_shared<MockSPIDevice::NiceMock>();

  // Return -1 to simulate failed setting bits
  ON_CALL(*spi_device, IOControl(testing::_, SPI_IOC_WR_BITS_PER_WORD, testing::_))
    .WillByDefault(testing::Return(-1));
  EXPECT_CALL(*spi_device, Close(testing::_)).Times(1);

  EXPECT_THROW(APA102Wrapper(spi_device, kMockDeviceName), std::ios_base::failure);
}

TEST(TestInitialization, SetSpeedFailure)
{
  auto spi_device = std::make_shared<MockSPIDevice::NiceMock>();

  // Return -1 to simulate failed setting speed
  ON_CALL(*spi_device, IOControl(testing::_, SPI_IOC_WR_MAX_SPEED_HZ, testing::_))
    .WillByDefault(testing::Return(-1));
  EXPECT_CALL(*spi_device, Close(testing::_)).Times(1);

  EXPECT_THROW(APA102Wrapper(spi_device, kMockDeviceName), std::ios_base::failure);
}

TEST_F(TestAPA102, SetGlobalBrightnessRatioNegative)
{
  const float brightness_ratio = -1.0;
  EXPECT_THROW(apa102_->SetGlobalBrightness(brightness_ratio), std::out_of_range);
}

TEST_F(TestAPA102, SetGlobalBrightnessRatioTooHigh)
{
  const float brightness_ratio = 1.1;
  EXPECT_THROW(apa102_->SetGlobalBrightness(brightness_ratio), std::out_of_range);
}

TEST_F(TestAPA102, SetGlobalBrightnessRatioValid)
{
  const float brightness_ratio = 0.5;

  ASSERT_NO_THROW(apa102_->SetGlobalBrightness(brightness_ratio));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 16);
}

TEST_F(TestAPA102, SetGlobalBrightnessNegative)
{
  // Wrap around to 255
  const std::uint8_t brightness = -1;

  EXPECT_THROW(apa102_->SetGlobalBrightness(brightness), std::out_of_range);
}

TEST_F(TestAPA102, SetGlobalBrightnessTooHigh)
{
  const std::uint8_t brightness = 32;

  EXPECT_THROW(apa102_->SetGlobalBrightness(brightness), std::out_of_range);
}

TEST_F(TestAPA102, SetGlobalBrightnessValid)
{
  const std::uint8_t brightness = 16;

  ASSERT_NO_THROW(apa102_->SetGlobalBrightness(brightness));
  EXPECT_EQ(apa102_->GetGlobalBrightness(), 16);
}

TEST_F(TestAPA102, RGBAFrameToBGRBufferInvalidFrame)
{
  std::vector<std::uint8_t> frame = {
    255, 128, 64};  // Valid frame requires 4 values to match RGBA format

  EXPECT_THROW(apa102_->RGBAFrameToBGRBuffer(frame), std::runtime_error);
}

TEST_F(TestAPA102, RGBAFrameToBGRBuffer)
{
  std::vector<std::uint8_t> frame = {255, 128, 64, 192};  // RGBA format

  apa102_->SetGlobalBrightness(std::uint8_t(16));
  auto buffer = apa102_->RGBAFrameToBGRBuffer(frame);

  ASSERT_EQ(buffer.size(), static_cast<size_t>(12));

  // Verify start frame
  for (int i = 0; i < 4; i++) {
    EXPECT_EQ(buffer[i], kStartFrame);
  }
  // Verify end frame
  for (int i = 8; i < 12; i++) {
    EXPECT_EQ(buffer[i], kEndFrame);
  }

  // Verify RGBA frame
  EXPECT_EQ(buffer[4], 0xEC);  // brightness value based on the frame
  EXPECT_EQ(buffer[5], 0x0B);  // B component after color correction
  EXPECT_EQ(buffer[6], 0x37);  // G component after color correction
  EXPECT_EQ(buffer[7], 0xF5);  // R component after color correction
}

TEST_F(TestAPA102, RGBAFrameToBGRBufferMatchesGammaFormula)
{
  // Full alpha and full global brightness, so every colour byte is the gamma correction alone.
  std::vector<std::uint8_t> frame;
  for (int v = 0; v < 256; v++) {
    frame.insert(frame.end(), {std::uint8_t(v), std::uint8_t(v), std::uint8_t(v), 255});
  }

  apa102_->SetGlobalBrightness(std::uint8_t(31));
  const auto buffer = apa102_->RGBAFrameToBGRBuffer(frame);

  const float gamma = 2.2;
  for (int v = 0; v < 256; v++) {
    const std::size_t led = 4 + 4 * v;
    EXPECT_EQ(buffer[led], 0xFF);
    EXPECT_EQ(buffer[led + 1], std::uint8_t(pow(v / 255.0, gamma) * 240)) << "blue " << v;
    EXPECT_EQ(buffer[led + 2], std::uint8_t(pow(v / 255.0, gamma) * 255)) << "green " << v;
    EXPECT_EQ(buffer[led + 3], std::uint8_t(pow(v / 255.0, gamma) * 245)) << "red " << v;
  }
}

class TestAPA102Transfers : public testing::Test
{
protected:
  // Sends one frame of num_led LEDs and returns the transfers the ioctl was given.
  std::vector<struct spi_ioc_transfer> SendFrame(
    const std::size_t num_led, const std::size_t max_transfer_bytes, const int ioctl_return = 1)
  {
    auto spi_device = std::make_shared<MockSPIDevice::NiceMock>();
    APA102Wrapper apa102(spi_device, kMockDeviceName, 4000000, max_transfer_bytes);

    std::vector<struct spi_ioc_transfer> transfers;
    expected_bytes_ = num_led * 4 + 8;
    ON_CALL(*spi_device, IOControl(testing::_, testing::_, testing::_))
      .WillByDefault(testing::Return(0));
    for (std::size_t n = 1; n <= 4; n++) {
      ON_CALL(*spi_device, IOControl(testing::_, SPI_IOC_MESSAGE(n), testing::_))
        .WillByDefault([&transfers, n, ioctl_return](int, unsigned long, const void * arg) {
          const auto * tr = static_cast<const struct spi_ioc_transfer *>(arg);
          transfers.assign(tr, tr + n);
          return ioctl_return;
        });
    }

    apa102.SetPanel(std::vector<std::uint8_t>(num_led * 4, 128));
    return transfers;
  }

  void ExpectContiguous(const std::vector<struct spi_ioc_transfer> & transfers)
  {
    std::size_t total = 0;
    for (std::size_t i = 0; i < transfers.size(); i++) {
      EXPECT_EQ(transfers[i].tx_buf, transfers[0].tx_buf + total);
      EXPECT_EQ(transfers[i].rx_buf, 0u);
      EXPECT_EQ(transfers[i].speed_hz, 4000000u);
      EXPECT_EQ(transfers[i].bits_per_word, 8);
      EXPECT_EQ(transfers[i].cs_change, 0);
      total += transfers[i].len;
    }
    EXPECT_EQ(total, expected_bytes_);
  }

  std::size_t expected_bytes_ = 0;
};

TEST_F(TestAPA102Transfers, WholeFrameByDefault)
{
  const auto transfers = SendFrame(24, 0);
  ASSERT_EQ(transfers.size(), 1u);
  EXPECT_EQ(transfers[0].len, 104u);
  ExpectContiguous(transfers);
}

TEST_F(TestAPA102Transfers, FrameShorterThanLimitIsOneTransfer)
{
  const auto transfers = SendFrame(24, 104);
  ASSERT_EQ(transfers.size(), 1u);
  ExpectContiguous(transfers);
}

TEST_F(TestAPA102Transfers, LynxFrameSplit)
{
  const auto transfers = SendFrame(24, 64);
  ASSERT_EQ(transfers.size(), 2u);
  EXPECT_EQ(transfers[0].len, 64u);
  EXPECT_EQ(transfers[1].len, 40u);
  ExpectContiguous(transfers);
}

TEST_F(TestAPA102Transfers, PantherFrameSplit)
{
  const auto transfers = SendFrame(46, 64);
  ASSERT_EQ(transfers.size(), 3u);
  for (const auto & tr : transfers) {
    EXPECT_EQ(tr.len, 64u);
  }
  ExpectContiguous(transfers);
}

TEST_F(TestAPA102Transfers, FailedTransferThrows)
{
  EXPECT_THROW(SendFrame(46, 64, -1), std::ios_base::failure);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);

  auto result = RUN_ALL_TESTS();
  return result;
}
