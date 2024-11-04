cmake_minimum_required(VERSION 3.10.2)
project(panther_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_updater
    image_transport
    panther_msgs
    panther_utils
    pluginlib
    rclcpp
    rclcpp_components
    sensor_msgs
    std_srvs
    yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

add_library(
  panther_animation_plugins SHARED src/animation/image_animation.cpp
                                   src/animation/charging_animation.cpp)
ament_target_dependencies(panther_animation_plugins panther_utils pluginlib)
target_link_libraries(panther_animation_plugins png yaml-cpp)

add_library(lights_driver_node_component SHARED src/lights_driver_node.cpp
                                                src/apa102.cpp)
ament_target_dependencies(
  lights_driver_node_component
  diagnostic_updater
  image_transport
  panther_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs)

add_library(
  lights_controller_node_component SHARED
  src/lights_controller_node.cpp src/led_components/led_segment.cpp
  src/led_components/led_panel.cpp src/led_components/segment_converter.cpp
  src/led_components/led_animations_queue.cpp)
ament_target_dependencies(
  lights_controller_node_component
  panther_msgs
  panther_utils
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs)
target_link_libraries(lights_controller_node_component yaml-cpp)

rclcpp_components_register_node(
  lights_driver_node_component PLUGIN "panther_lights::LightsDriverNode"
  EXECUTABLE lights_driver_node)

rclcpp_components_register_node(
  lights_controller_node_component PLUGIN
  "panther_lights::LightsControllerNode" EXECUTABLE lights_controller_node)

ament_export_targets(export_lights_driver_node_component)
install(
  TARGETS lights_driver_node_component
  EXPORT export_lights_driver_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_export_targets(export_lights_controller_node_component)
install(
  TARGETS lights_controller_node_component
  EXPORT export_lights_controller_node_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS panther_animation_plugins LIBRARY DESTINATION lib)

install(DIRECTORY animations config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)

  # Unit tests
  ament_add_gtest(${PROJECT_NAME}_test_animation
                  test/unit/animation/test_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_animation panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_charging_animation
    test/unit/animation/test_charging_animation.cpp
    src/animation/charging_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_charging_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_charging_animation
                            ament_index_cpp panther_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_charging_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_image_animation
    test/unit/animation/test_image_animation.cpp
    src/animation/image_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_image_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_image_animation
                            ament_index_cpp panther_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_image_animation png yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_panel test/unit/led_components/test_led_panel.cpp
    src/led_components/led_panel.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_panel
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_segment
    test/unit/led_components/test_led_segment.cpp
    src/led_components/led_segment.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_segment
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_segment pluginlib
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_led_segment yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_segment_converter
    test/unit/led_components/test_segment_converter.cpp
    src/led_components/segment_converter.cpp
    src/led_components/led_panel.cpp
    src/led_components/led_segment.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_segment_converter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_segment_converter pluginlib
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_segment_converter yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_animation
    test/unit/led_components/test_led_animation.cpp
    src/led_components/led_panel.cpp
    src/led_components/led_segment.cpp
    src/led_components/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_animation pluginlib
                            panther_utils rclcpp)
  target_link_libraries(${PROJECT_NAME}_test_led_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_animations_queue
    test/unit/led_components/test_led_animations_queue.cpp
    src/led_components/led_panel.cpp
    src/led_components/led_segment.cpp
    src/led_components/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_animations_queue
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_animations_queue pluginlib
                            panther_utils rclcpp)
  target_link_libraries(${PROJECT_NAME}_test_led_animations_queue yaml-cpp)

  ament_add_gmock(${PROJECT_NAME}_test_apa102 test/unit/test_apa102.cpp
                  src/apa102.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_apa102
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gmock(
    ${PROJECT_NAME}_test_lights_driver_node
    test/unit/test_lights_driver_node.cpp src/apa102.cpp
    src/lights_driver_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_driver_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_lights_driver_node
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_lights_driver_node
                        lights_driver_node_component)

  ament_add_gtest(
    ${PROJECT_NAME}_test_lights_controller_node
    test/unit/test_lights_controller_node.cpp
    src/lights_controller_node.cpp
    src/led_components/led_segment.cpp
    src/led_components/led_panel.cpp
    src/led_components/segment_converter.cpp
    src/led_components/led_animations_queue.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_controller_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_lights_controller_node
                            panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_lights_controller_node
                        lights_controller_node_component yaml-cpp)

  # Integration tests
  option(TEST_INTEGRATION "Run integration tests" OFF)
  if(TEST_INTEGRATION)
    add_ros_test(test/integration/panther_lights.test.py)
  endif()

endif()

ament_export_include_directories(include)
ament_export_libraries(panther_animation_plugins)

ament_package()
