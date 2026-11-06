cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake diagnostic_msgs diagnostic_updater
                         husarion_ugv_utils rclcpp sensor_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_executable(resilient_joy_linux src/resilient_joy_linux.cpp)
target_include_directories(resilient_joy_linux PRIVATE include)
ament_target_dependencies(
  resilient_joy_linux diagnostic_msgs diagnostic_updater husarion_ugv_utils
  rclcpp sensor_msgs)

install(TARGETS resilient_joy_linux DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_gamepad_diagnostics
                  test/test_gamepad_diagnostics.cpp)
  target_include_directories(${PROJECT_NAME}_test_gamepad_diagnostics
                             PRIVATE include)
  ament_target_dependencies(${PROJECT_NAME}_test_gamepad_diagnostics
                            diagnostic_msgs)
endif()

ament_package()
