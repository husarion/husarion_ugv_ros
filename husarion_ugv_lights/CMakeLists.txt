cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_lights)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    diagnostic_updater
    generate_parameter_library
    image_transport
    husarion_ugv_msgs
    husarion_ugv_utils
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

add_library(animation_plugins SHARED src/animation/image_animation.cpp
                                     src/animation/moving_image_animation.cpp)

ament_target_dependencies(animation_plugins husarion_ugv_utils pluginlib)
target_link_libraries(animation_plugins png yaml-cpp)

add_library(lights_driver_node_component SHARED src/lights_driver_node.cpp
                                                src/apa102.cpp)
ament_target_dependencies(
  lights_driver_node_component
  diagnostic_updater
  image_transport
  husarion_ugv_msgs
  rclcpp
  rclcpp_components
  sensor_msgs
  std_srvs)

generate_parameter_library(lights_driver_parameters
                           src/lights_driver_parameters.yaml)
target_link_libraries(lights_driver_node_component lights_driver_parameters)

add_library(
  lights_controller_node_component SHARED
  src/lights_controller_node.cpp
  src/led_components/led_segment.cpp
  src/led_components/led_panel.cpp
  src/led_components/segment_converter.cpp
  src/led_components/segment_layer.cpp
  src/led_components/segment_queue_layer.cpp)
ament_target_dependencies(
  lights_controller_node_component
  husarion_ugv_msgs
  husarion_ugv_utils
  pluginlib
  rclcpp
  rclcpp_components
  sensor_msgs)

generate_parameter_library(lights_controller_parameters
                           src/lights_controller_parameters.yaml)
target_link_libraries(lights_controller_node_component yaml-cpp
                      lights_controller_parameters)

rclcpp_components_register_node(
  lights_driver_node_component PLUGIN "husarion_ugv_lights::LightsDriverNode"
  EXECUTABLE lights_driver_node)

rclcpp_components_register_node(
  lights_controller_node_component PLUGIN
  "husarion_ugv_lights::LightsControllerNode" EXECUTABLE lights_controller_node)

install(
  TARGETS animation_plugins lights_driver_node_component
          lights_driver_parameters lights_controller_node_component
          lights_controller_parameters
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY animations config launch test
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

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
  ament_target_dependencies(${PROJECT_NAME}_test_animation husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_animation yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_image_animation
    test/unit/animation/test_image_animation.cpp
    src/animation/image_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_image_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_image_animation
                            ament_index_cpp husarion_ugv_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_image_animation png yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_moving_image_animation
    test/unit/animation/test_moving_image_animation.cpp
    src/animation/moving_image_animation.cpp src/animation/image_animation.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_moving_image_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_moving_image_animation
                            ament_index_cpp husarion_ugv_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_moving_image_animation png
                        yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_panel test/unit/led_components/test_led_panel.cpp
    src/led_components/led_panel.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_panel
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_segment_layer_interface
                  test/unit/led_components/test_segment_layer_interface.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_segment_layer_interface
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_segment_layer_interface
                            husarion_ugv_utils pluginlib)
  target_link_libraries(${PROJECT_NAME}_test_segment_layer_interface yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_segment
    test/unit/led_components/test_led_segment.cpp
    src/led_components/led_segment.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_segment
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_segment pluginlib
                            husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_led_segment yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_segment_converter
    test/unit/led_components/test_segment_converter.cpp
    src/led_components/segment_converter.cpp
    src/led_components/led_panel.cpp
    src/led_components/led_segment.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_segment_converter
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_segment_converter pluginlib
                            husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_segment_converter yaml-cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_led_animation
    test/unit/led_components/test_led_animation.cpp
    src/led_components/led_panel.cpp
    src/led_components/led_segment.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_led_animation
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_led_animation pluginlib
                            husarion_ugv_utils rclcpp)
  target_link_libraries(${PROJECT_NAME}_test_led_animation yaml-cpp)

  ament_add_gmock(${PROJECT_NAME}_test_apa102 test/unit/test_apa102.cpp
                  src/apa102.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_apa102
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gmock(
    ${PROJECT_NAME}_test_lights_driver_node
    test/unit/test_lights_driver_node.cpp
    src/apa102.cpp
    src/lights_driver_node.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_driver_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_lights_driver_node pluginlib
                            husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_lights_driver_node
                        lights_driver_node_component)

  ament_add_gtest(
    ${PROJECT_NAME}_test_lights_controller_node
    test/unit/test_lights_controller_node.cpp
    src/lights_controller_node.cpp
    src/led_components/led_segment.cpp
    src/led_components/led_panel.cpp
    src/led_components/segment_converter.cpp
    src/led_components/segment_layer.cpp
    src/led_components/segment_queue_layer.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_controller_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_lights_controller_node
                            pluginlib husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_lights_controller_node
                        lights_controller_node_component yaml-cpp)

  # Integration tests
  option(TEST_INTEGRATION "Run integration tests" OFF)
  if(TEST_INTEGRATION)
    add_ros_test(test/integration/husarion_ugv_lights.test.py)
  endif()

endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_package()
