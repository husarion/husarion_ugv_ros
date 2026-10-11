cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES ament_cmake diagnostic_updater husarion_ugv_utils
                         rclcpp sensor_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

add_executable(resilient_joy_linux src/resilient_joy_linux.cpp)
ament_target_dependencies(resilient_joy_linux diagnostic_updater
                          husarion_ugv_utils rclcpp sensor_msgs)

install(TARGETS resilient_joy_linux DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
