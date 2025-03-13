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

#ifndef HUSARION_UGV_LIGHTS_APA102_HPP_
#define HUSARION_UGV_LIGHTS_APA102_HPP_

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace husarion_ugv_lights
{

class SPIDeviceInterface
{
public:
  virtual ~SPIDeviceInterface() = default;

  /**
   * @brief Open SPI device
   *
   * @param device Name of the device
   */
  virtual int Open(const std::string & device) = 0;

  /**
   * @brief Perform an I/O control operation on the device
   *
   * @param fd File descriptor
   * @param request Request code
   * @param arg Argument
   */
  virtual int IOControl(int fd, unsigned long request, const void * arg) = 0;

  /**
   * @brief Close the device
   *
   * @param fd File descriptor
   */
  virtual int Close(int fd) = 0;

  using SharedPtr = std::shared_ptr<SPIDeviceInterface>;
};

class SPIDevice : public SPIDeviceInterface
{
public:
  int Open(const std::string & device) override { return ::open(device.c_str(), O_WRONLY); }

  int IOControl(int fd, unsigned long request, const void * arg) override
  {
    return ::ioctl(fd, request, arg);
  }

  int Close(int fd) override { return ::close(fd); }
};

class APA102Interface
{
public:
  virtual ~APA102Interface() = default;

  virtual void SetGlobalBrightness(const std::uint8_t brightness) = 0;
  virtual void SetGlobalBrightness(const float brightness) = 0;
  virtual void SetPanel(const std::vector<std::uint8_t> & frame) const = 0;

  using SharedPtr = std::shared_ptr<APA102Interface>;
};

/**
 * @brief Class representing an APA102 LED panel.
 *
 * This class provides methods to control the APA102 LED panel, including setting the global
 * brightness, setting the LED panel based on a given frame.
 *
 * @param spi_device SPI Device object
 * @param device_name name of the device
 * @param speed Speed of the SPI communication
 * @param cs_high Chip select high flag
 */
class APA102 : public APA102Interface
{
public:
  APA102(
    SPIDeviceInterface::SharedPtr spi_device, const std::string & device_name,
    const std::uint32_t speed = 800000, const bool cs_high = false);
  ~APA102();

  /**
   * @brief Set APA102 LED global brightness
   *
   * @param brightness value in range from 0 to 31
   *
   * @exception std::out_of_range if brightness value is out of defined range
   */
  void SetGlobalBrightness(const std::uint8_t brightness) override;

  /**
   * @brief Set APA102 LED global brightness
   *
   * @param brightness value in range from 0.0 to 1.0
   *
   * @exception std::out_of_range if brightness value is out of defined range
   */
  void SetGlobalBrightness(const float brightness) override;

  /**
   * @brief Set APA102 LED panel based on given frame
   *
   * @param frame vector in the RGBA format where alpha represents brightness of a given LED
   *
   * @exception std::ios_base::failure if failed to send data over SPI
   * or std::runtime_error if frame is invalid
   */
  void SetPanel(const std::vector<std::uint8_t> & frame) const override;

protected:
  /**
   * @brief Creates buffer with BGR format with structure appropriate for
   * the SPI transfer based on a given RGBA frame
   *
   * @return buffer vector.
   *
   * @exception std::runtime_error if frame has incorrect number of bytes
   */
  std::vector<std::uint8_t> RGBAFrameToBGRBuffer(const std::vector<std::uint8_t> & frame) const;

  /**
   * @brief Create transfer object based on buffer and send data over SPI
   *
   * @exception std::ios_base::failure if failed to send data over SPI
   */
  void SPISendBuffer(const std::vector<std::uint8_t> & buffer) const;

  std::uint16_t global_brightness_;

private:
  static constexpr std::uint8_t kBits = 8;

  // Color correction constants, found experimentally
  static constexpr std::uint16_t kCorrRed = 245;
  static constexpr std::uint16_t kCorrGreen = 255;
  static constexpr std::uint16_t kCorrBlue = 240;
  static constexpr float kCorrectionGamma = 2.2;

  SPIDeviceInterface::SharedPtr spi_device_;
  const std::string device_name_;
  const std::uint32_t speed_;
  const int file_descriptor_;
};

}  // namespace husarion_ugv_lights

#endif  // HUSARION_UGV_LIGHTS_APA102_HPP_
