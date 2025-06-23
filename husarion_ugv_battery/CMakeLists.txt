cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_battery)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_updater
    generate_parameter_library
    husarion_ugv_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include ${husarion_ugv_utils_INCLUDE_DIRS})

add_executable(
  battery_driver_node
  src/main.cpp
  src/battery_driver_node.cpp
  src/battery/adc_battery.cpp
  src/battery/roboteq_battery.cpp
  src/battery_publisher/battery_publisher.cpp
  src/battery_publisher/dual_battery_publisher.cpp
  src/battery_publisher/single_battery_publisher.cpp)
ament_target_dependencies(battery_driver_node ${PACKAGE_DEPENDENCIES})

generate_parameter_library(battery_parameters src/battery_parameters.yaml)
target_link_libraries(battery_driver_node battery_parameters)

install(TARGETS battery_driver_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_adc_data_reader
                  test/test_adc_data_reader.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_data_reader
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_battery test/battery/test_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery rclcpp sensor_msgs
                            husarion_ugv_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_adc_battery
                  test/battery/test_adc_battery.cpp src/battery/adc_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_adc_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_adc_battery rclcpp sensor_msgs
                            husarion_ugv_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_battery test/battery/test_roboteq_battery.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_battery
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_battery
                            husarion_ugv_msgs rclcpp sensor_msgs)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_publisher
    test/battery_publisher/test_battery_publisher.cpp
    src/battery_publisher/battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_publisher
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_battery_publisher
                        battery_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_single_battery_publisher
    test/battery_publisher/test_single_battery_publisher.cpp
    src/battery/adc_battery.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_single_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_single_battery_publisher
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_single_battery_publisher
                        battery_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_dual_battery_publisher
    test/battery_publisher/test_dual_battery_publisher.cpp
    src/battery/adc_battery.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_dual_battery_publisher
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_dual_battery_publisher
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_dual_battery_publisher
                        battery_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_adc_dual
    test/test_battery_driver_node_adc_dual.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_adc_dual
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_adc_dual
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_battery_driver_node_adc_dual
                        battery_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_adc_single
    test/test_battery_driver_node_adc_single.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_adc_single
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_adc_single
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_battery_driver_node_adc_single
                        battery_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_battery_driver_node_roboteq
    test/test_battery_driver_node_roboteq.cpp
    src/battery/adc_battery.cpp
    src/battery_driver_node.cpp
    src/battery_publisher/battery_publisher.cpp
    src/battery_publisher/dual_battery_publisher.cpp
    src/battery_publisher/single_battery_publisher.cpp
    src/battery/roboteq_battery.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_battery_driver_node_roboteq
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_battery_driver_node_roboteq
                            ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_battery_driver_node_roboteq
                        battery_parameters)

endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
