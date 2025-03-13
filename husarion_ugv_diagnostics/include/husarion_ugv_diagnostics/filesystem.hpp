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

#ifndef HUSARION_UGV_DIAGNOSTICS_FILESYSTEM_HPP_
#define HUSARION_UGV_DIAGNOSTICS_FILESYSTEM_HPP_

#include <filesystem>
#include <fstream>

namespace husarion_ugv_diagnostics
{

/**
 * @brief Abstract interface for the filesystem methods.
 */
class FilesystemInterface
{
public:
  /**
   * @brief Virtual destructor for the FilesystemInterface class.
   */
  virtual ~FilesystemInterface() = default;

  virtual uintmax_t GetSpaceCapacity(const std::string & filesystem_path) const = 0;

  virtual uintmax_t GetSpaceAvailable(const std::string & filesystem_path) const = 0;

  virtual std::string ReadFile(const std::string & file_path) const = 0;

  /**
   * @brief Alias for a shared pointer to a FilesystemInterface object.
   */
  using SharedPtr = std::shared_ptr<FilesystemInterface>;
};

/**
 * @brief A class that provides functionality for interacting with the filesystem.
 *
 * This class inherits from the `FilesystemInterface` and implements its methods.
 * It provides a simplified, facade-type way to interact with the `std::filesystem` library.
 */
class Filesystem : public FilesystemInterface
{
public:
  /**
   * @brief Returns the space capacity in bytes of the filesystem at the specified path.
   *
   * @param filesystem_path The path to the filesystem.
   * @return The space capacity in bytes.
   */
  inline uintmax_t GetSpaceCapacity(const std::string & filesystem_path) const override
  {
    const auto path = std::filesystem::path(filesystem_path);
    const auto space_info = std::filesystem::space(path);

    return space_info.capacity;
  }

  /**
   * @brief Returns the available space in bytes of the filesystem at the specified path.
   *
   * @param filesystem_path The path to the filesystem.
   * @return The available space in bytes.
   */
  inline uintmax_t GetSpaceAvailable(const std::string & filesystem_path) const override
  {
    const auto path = std::filesystem::path(filesystem_path);
    const auto space_info = std::filesystem::space(path);

    return space_info.available;
  }

  /**
   * @brief Reads the contents of the file specified by the given file path.
   *
   * @param file_path The path to the file to be read.
   * @return The contents of the file as a string.
   * @throws `std::invalid_argument` If the file doesn't exist.
   * @throws `std::runtime_error` If the file fails to open.
   */
  std::string ReadFile(const std::string & file_path) const override
  {
    const auto path = std::filesystem::path(file_path);

    if (!std::filesystem::exists(path)) {
      throw std::invalid_argument("File doesn't exist, given path " + path.string());
    }

    std::ifstream file(path);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open, given path " + path.string());
    }

    std::stringstream buffer;
    buffer << file.rdbuf();

    file.close();
    return buffer.str();
  }
};

}  // namespace husarion_ugv_diagnostics

#endif  // HUSARION_UGV_DIAGNOSTICS_FILESYSTEM_HPP_
