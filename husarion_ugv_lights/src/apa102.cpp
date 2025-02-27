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

#include "husarion_ugv_lights/apa102.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace husarion_ugv_lights
{

APA102::APA102(
  SPIDeviceInterface::SharedPtr spi_device, const std::string & device_name,
  const std::uint32_t speed, const bool cs_high)
: spi_device_(spi_device),
  device_name_(device_name),
  speed_(speed),
  file_descriptor_(spi_device->Open(device_name))
{
  if (file_descriptor_ < 0) {
    throw std::ios_base::failure("Failed to open " + device_name_ + ".");
  }

  static std::uint8_t mode = cs_high ? SPI_MODE_3 : SPI_MODE_3 | SPI_CS_HIGH;
  if (spi_device_->IOControl(file_descriptor_, SPI_IOC_WR_MODE32, &mode) == -1) {
    spi_device_->Close(file_descriptor_);
    throw std::ios_base::failure("Failed to set mode for " + device_name_ + ".");
  }

  if (spi_device_->IOControl(file_descriptor_, SPI_IOC_WR_BITS_PER_WORD, &kBits) == -1) {
    spi_device_->Close(file_descriptor_);
    throw std::ios_base::failure("Can't set bits per word for " + device_name_ + ".");
  }

  if (spi_device_->IOControl(file_descriptor_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_) == -1) {
    spi_device_->Close(file_descriptor_);
    throw std::ios_base::failure("Can't set speed for " + device_name_ + ".");
  }
}

APA102::~APA102() { spi_device_->Close(file_descriptor_); }

void APA102::SetGlobalBrightness(const float brightness)
{
  if (brightness < 0.0f || brightness > 1.0f) {
    throw std::out_of_range("Brightness out of range [0.0, 1.0].");
  }
  SetGlobalBrightness(std::uint8_t(ceil(brightness * 31.0f)));
}

void APA102::SetGlobalBrightness(const std::uint8_t brightness)
{
  if (brightness > 31) {
    throw std::out_of_range("Brightness out of range [0, 31].");
  }
  global_brightness_ = std::uint16_t(brightness);
}

void APA102::SetPanel(const std::vector<std::uint8_t> & frame) const
{
  const auto buffer = RGBAFrameToBGRBuffer(frame);
  SPISendBuffer(buffer);
}

std::vector<std::uint8_t> APA102::RGBAFrameToBGRBuffer(
  const std::vector<std::uint8_t> & frame) const
{
  if (frame.size() % 4 != 0) {
    throw std::runtime_error("Incorrect number of bytes to convert frame.");
  }

  const std::size_t buffer_size = (4 * sizeof(std::uint8_t)) + frame.size() +
                                  (4 * sizeof(std::uint8_t));
  std::vector<std::uint8_t> buffer(buffer_size);

  // Init start and end frames
  std::fill(buffer.begin(), buffer.begin() + 4, 0x00);
  std::fill(buffer.end() - 4, buffer.end(), 0xFF);

  // Copy frame from vector to sending buffer
  for (std::size_t i = 0; i < frame.size() / 4; i++) {
    const std::size_t padding = i * 4;
    // Header with brightness
    const std::uint8_t brightness = (std::uint16_t(frame[padding + 3]) * global_brightness_) / 255;
    buffer[4 + padding] = 0xE0 | brightness;
    // Convert rgb to bgr with color correction
    buffer[4 + padding + 1] =
      std::uint8_t(pow(frame[padding + 2] / 255.0, kCorrectionGamma) * kCorrBlue);
    buffer[4 + padding + 2] =
      std::uint8_t(pow(frame[padding + 1] / 255.0, kCorrectionGamma) * kCorrGreen);
    buffer[4 + padding + 3] =
      std::uint8_t(pow(frame[padding + 0] / 255.0, kCorrectionGamma) * kCorrRed);
  }

  return buffer;
}

void APA102::SPISendBuffer(const std::vector<std::uint8_t> & buffer) const
{
  struct spi_ioc_transfer tr;
  memset(&tr, 0, sizeof(tr));
  tr.tx_buf = reinterpret_cast<std::uint64_t>(buffer.data());
  tr.rx_buf = 0;
  tr.len = static_cast<std::uint32_t>(buffer.size());
  tr.speed_hz = speed_;
  tr.delay_usecs = 0;
  tr.bits_per_word = kBits;

  const int ret = spi_device_->IOControl(file_descriptor_, SPI_IOC_MESSAGE(1), &tr);

  if (ret < 1) {
    throw std::ios_base::failure("Failed to send data over SPI " + device_name_ + ".");
  }
}

}  // namespace husarion_ugv_lights
