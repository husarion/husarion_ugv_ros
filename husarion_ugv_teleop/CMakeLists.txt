cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake diagnostic_msgs diagnostic_updater
                         husarion_ugv_utils rclcpp sensor_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_executable(resilient_joy_linux_node src/resilient_joy_linux.cpp)
target_include_directories(resilient_joy_linux_node PRIVATE include)
# The vendored joy node keeps upstream's C casts.
target_compile_options(resilient_joy_linux_node PRIVATE -Wno-old-style-cast)
ament_target_dependencies(
  resilient_joy_linux_node diagnostic_msgs diagnostic_updater
  husarion_ugv_utils rclcpp sensor_msgs)

install(TARGETS resilient_joy_linux_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ros_testing REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_gamepad_utils
                  test/test_gamepad_utils.cpp)
  target_include_directories(${PROJECT_NAME}_test_gamepad_utils PRIVATE include)
  ament_target_dependencies(${PROJECT_NAME}_test_gamepad_utils diagnostic_msgs)

  add_ros_test(test/resilient_joy_linux_shutdown.test.py TIMEOUT 60)
  add_ros_test(test/resilient_joy_linux_reconnect_shutdown.test.py TIMEOUT 60)
endif()

ament_package()
