cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

install(
  PROGRAMS scripts/nmea_socket_driver.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nmea_socket_driver)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(${PROJECT_NAME}_test_nmea_socket_driver
                        test/test_nmea_socket_driver.py)
endif()

ament_package()
