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

#include <algorithm>
#include <cmath>
#include <cstddef>
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
  const std::uint32_t speed, const bool cs_high, const std::size_t max_transfer_bytes)
: spi_device_(spi_device),
  device_name_(device_name),
  speed_(speed),
  max_transfer_bytes_(max_transfer_bytes),
  file_descriptor_(spi_device->Open(device_name))
{
  if (file_descriptor_ < 0) {
    throw std::ios_base::failure("Failed to open " + device_name_ + ".");
  }

  // SPI_IOC_WR_MODE32 reads four bytes, so the mode has to be a 32-bit word. The mapping looks
  // inverted (cs_high false requests SPI_CS_HIGH), but it is what the strips have always run with.
  const std::uint32_t mode = cs_high ? SPI_MODE_3 : SPI_MODE_3 | SPI_CS_HIGH;
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

  const auto & luts = GetGammaLUTs();

  // Copy frame from vector to sending buffer
  for (std::size_t i = 0; i < frame.size() / 4; i++) {
    const std::size_t padding = i * 4;
    // Header with brightness
    const std::uint8_t brightness = (std::uint16_t(frame[padding + 3]) * global_brightness_) / 255;
    buffer[4 + padding] = 0xE0 | brightness;
    // Convert rgb to bgr with color correction
    buffer[4 + padding + 1] = luts.blue[frame[padding + 2]];
    buffer[4 + padding + 2] = luts.green[frame[padding + 1]];
    buffer[4 + padding + 3] = luts.red[frame[padding + 0]];
  }

  return buffer;
}

const APA102::GammaLUTs & APA102::GetGammaLUTs()
{
  static const GammaLUTs luts = []() {
    GammaLUTs l;
    for (std::size_t i = 0; i < 256; i++) {
      l.red[i] = std::uint8_t(pow(i / 255.0, kCorrectionGamma) * kCorrRed);
      l.green[i] = std::uint8_t(pow(i / 255.0, kCorrectionGamma) * kCorrGreen);
      l.blue[i] = std::uint8_t(pow(i / 255.0, kCorrectionGamma) * kCorrBlue);
    }
    return l;
  }();
  return luts;
}

std::size_t APA102::NumberOfTransfers(
  const std::size_t num_led, const std::size_t max_transfer_bytes)
{
  // Start frame, four bytes per LED, end frame - the buffer RGBAFrameToBGRBuffer builds.
  return NumberOfTransfersForBuffer(4 + 4 * num_led + 4, max_transfer_bytes);
}

std::size_t APA102::NumberOfTransfersForBuffer(
  const std::size_t buffer_size, const std::size_t max_transfer_bytes)
{
  if (max_transfer_bytes == 0) {
    return 1;
  }
  return std::max<std::size_t>(1, (buffer_size + max_transfer_bytes - 1) / max_transfer_bytes);
}

void APA102::SPISendBuffer(const std::vector<std::uint8_t> & buffer) const
{
  // spi-bcm2835 sends a transfer of 96 bytes or more by DMA, and its DMA completion busy-waits in
  // the IRQ thread until the FIFO drains, up to 0.64 ms per frame at 800 kHz. Shorter transfers go
  // out interrupt driven, or busy-polled in this thread when the driver expects them to take less
  // than its polling limit (30 us by default), which a high clock can reach. An APA102 strip has no
  // chip select and only follows the clock, so a pause in the clock between the transfers of one
  // message does not split the frame.
  const std::size_t chunk = max_transfer_bytes_ > 0 ? max_transfer_bytes_ : buffer.size();
  const std::size_t num_transfers = NumberOfTransfersForBuffer(buffer.size(), max_transfer_bytes_);
  if (num_transfers > kMaxSPITransfers) {
    throw std::ios_base::failure("Too many SPI transfers for one frame on " + device_name_ + ".");
  }

  std::vector<struct spi_ioc_transfer> transfers(num_transfers);
  for (std::size_t i = 0; i < num_transfers; i++) {
    auto & tr = transfers[i];
    memset(&tr, 0, sizeof(tr));
    const std::size_t offset = i * chunk;
    tr.tx_buf = reinterpret_cast<std::uint64_t>(buffer.data() + offset);
    tr.rx_buf = 0;
    tr.len = static_cast<std::uint32_t>(std::min(chunk, buffer.size() - offset));
    tr.speed_hz = speed_;
    tr.delay_usecs = 0;
    tr.bits_per_word = kBits;
  }

  // SPI_IOC_MESSAGE() with a runtime count would be a variable length array, so spell it out.
  const unsigned long request = _IOC(_IOC_WRITE, SPI_IOC_MAGIC, 0, SPI_MSGSIZE(num_transfers));
  const int ret = spi_device_->IOControl(file_descriptor_, request, transfers.data());

  if (ret < 1) {
    throw std::ios_base::failure("Failed to send data over SPI " + device_name_ + ".");
  }
}

std::uint32_t EffectiveSPISpeed(const std::uint64_t core_clock_hz, const std::uint32_t requested_hz)
{
  // Mirrors bcm2835_spi_transfer_one(): an even divider of at least 2, or 65536 when the request
  // is too slow for any divider the register holds.
  std::uint64_t divider = 65536;
  if (requested_hz >= core_clock_hz / 2) {
    divider = 2;
  } else if (requested_hz > 0) {
    divider = (core_clock_hz + requested_hz - 1) / requested_hz;
    divider += divider % 2;
    if (divider >= 65536) {
      divider = 65536;
    }
  }
  return static_cast<std::uint32_t>(core_clock_hz / divider);
}

}  // namespace husarion_ugv_lights
