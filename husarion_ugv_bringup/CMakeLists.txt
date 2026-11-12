cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

# The test runs the hardware bringup, so it needs the hardware packages
# installed.
if(BUILD_TESTING AND "$ENV{HUSARION_ROS_BUILD_TYPE}" STREQUAL "hardware")
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_test_bringup_launch
                        test/test_bringup_launch.py TIMEOUT 120)
endif()

ament_package()
