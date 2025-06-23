cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    generate_parameter_library
    geometry_msgs
    husarion_ugv_msgs
    husarion_ugv_utils
    OpenSSL
    rclcpp
    rclcpp_action
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs
    yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

add_library(call_set_bool_service_bt_node SHARED
            src/plugins/action/call_set_bool_service_node.cpp)
list(APPEND plugin_libs call_set_bool_service_bt_node)

add_library(call_trigger_service_bt_node SHARED
            src/plugins/action/call_trigger_service_node.cpp)
list(APPEND plugin_libs call_trigger_service_bt_node)

add_library(call_set_led_animation_service_bt_node SHARED
            src/plugins/action/call_set_led_animation_service_node.cpp)
list(APPEND plugin_libs call_set_led_animation_service_bt_node)

add_library(signal_shutdown_bt_node SHARED
            src/plugins/action/signal_shutdown_node.cpp)
list(APPEND plugin_libs signal_shutdown_bt_node)

add_library(shutdown_hosts_from_file_bt_node SHARED
            src/plugins/action/shutdown_hosts_from_file_node.cpp)
target_link_libraries(shutdown_hosts_from_file_bt_node OpenSSL::SSL yaml-cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_bt_node)

add_library(check_bool_msg_bt_node SHARED
            src/plugins/condition/check_bool_msg.cpp)
list(APPEND plugin_libs check_bool_msg_bt_node)

add_library(check_joy_msg_bt_node SHARED
            src/plugins/condition/check_joy_msg.cpp)
list(APPEND plugin_libs check_joy_msg_bt_node)

add_library(tick_after_timeout_bt_node SHARED
            src/plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)

add_library(execute_command_bt_node SHARED
            src/plugins/action/execute_command_node.cpp)
list(APPEND plugin_libs execute_command_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

add_executable(safety_manager_node src/safety_manager_node_main.cpp
                                   src/safety_manager_node.cpp)
ament_target_dependencies(
  safety_manager_node
  behaviortree_ros2
  geometry_msgs
  husarion_ugv_msgs
  husarion_ugv_utils
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs)

generate_parameter_library(safety_manager_parameters
                           src/safety_manager_parameters.yaml)
target_link_libraries(safety_manager_node ${plugin_libs}
                      safety_manager_parameters)

add_executable(lights_manager_node src/lights_manager_node_main.cpp
                                   src/lights_manager_node.cpp)
ament_target_dependencies(
  lights_manager_node
  behaviortree_ros2
  geometry_msgs
  husarion_ugv_msgs
  husarion_ugv_utils
  rclcpp
  sensor_msgs
  std_msgs
  tf2_geometry_msgs)

generate_parameter_library(lights_manager_parameters
                           src/lights_manager_parameters.yaml)
target_link_libraries(lights_manager_node ${plugin_libs}
                      lights_manager_parameters)

install(TARGETS ${plugin_libs} DESTINATION lib)

install(TARGETS safety_manager_node lights_manager_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

# Access via `husarion_ugv_manager/test/**` instead of `test/**`
install(DIRECTORY test/utils/
        DESTINATION include/${PROJECT_NAME}/${PROJECT_NAME}/test/utils)

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(PkgConfig REQUIRED)

  ament_add_gtest(
    ${PROJECT_NAME}_test_call_set_bool_service_node
    test/plugins/action/test_call_set_bool_service_node.cpp
    src/plugins/action/call_set_bool_service_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_call_set_bool_service_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_call_trigger_service_node
    test/plugins/action/test_call_trigger_service_node.cpp
    src/plugins/action/call_trigger_service_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_call_trigger_service_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_call_set_led_animation_service_node
    test/plugins/action/test_call_set_led_animation_service_node.cpp
    src/plugins/action/call_set_led_animation_service_node.cpp)
  list(APPEND plugin_tests
       ${PROJECT_NAME}_test_call_set_led_animation_service_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_execute_command_node
    test/plugins/action/test_execute_command_node.cpp
    src/plugins/action/execute_command_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_execute_command_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_check_bool_msg
    test/plugins/condition/test_check_bool_msg.cpp
    src/plugins/condition/check_bool_msg.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_check_bool_msg)

  ament_add_gtest(
    ${PROJECT_NAME}_test_check_joy_msg
    test/plugins/condition/test_check_joy_msg.cpp
    src/plugins/condition/check_joy_msg.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_check_joy_msg)

  ament_add_gtest(
    ${PROJECT_NAME}_test_tick_after_timeout_node
    test/plugins/decorator/test_tick_after_timeout_node.cpp
    src/plugins/decorator/tick_after_timeout_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_tick_after_timeout_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_signal_shutdown_node
    test/plugins/action/test_signal_shutdown_node.cpp
    src/plugins/action/signal_shutdown_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_signal_shutdown_node)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_host
                  test/plugins/test_shutdown_host.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_host)
  target_link_libraries(${PROJECT_NAME}_test_shutdown_host OpenSSL::SSL)

  ament_add_gmock(${PROJECT_NAME}_test_shutdown_hosts_node
                  test/plugins/test_shutdown_hosts_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_node)
  target_link_libraries(${PROJECT_NAME}_test_shutdown_hosts_node OpenSSL::SSL)

  ament_add_gtest(
    ${PROJECT_NAME}_test_shutdown_hosts_from_file_node
    test/plugins/action/test_shutdown_hosts_from_file_node.cpp
    src/plugins/action/shutdown_hosts_from_file_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_from_file_node)
  target_link_libraries(${PROJECT_NAME}_test_shutdown_hosts_from_file_node
                        OpenSSL::SSL)

  foreach(bt_node_test ${plugin_tests})
    target_include_directories(
      ${bt_node_test} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
                             $<INSTALL_INTERFACE:include>)

    ament_target_dependencies(${bt_node_test} ${PACKAGE_DEPENDENCIES})
    target_link_libraries(${bt_node_test} yaml-cpp)
  endforeach()

  ament_add_gtest(${PROJECT_NAME}_test_behavior_tree_utils
                  test/test_behavior_tree_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_behavior_tree_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_behavior_tree_utils behaviortree_cpp behaviortree_ros2
    husarion_ugv_utils geometry_msgs tf2_geometry_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_behavior_tree_manager
                  test/test_behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_behavior_tree_manager
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_behavior_tree_manager
                            behaviortree_cpp husarion_ugv_utils)

  ament_add_gtest(${PROJECT_NAME}_test_lights_manager_node
                  test/test_lights_manager_node.cpp src/lights_manager_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_manager_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_lights_manager_node
    behaviortree_cpp
    behaviortree_ros2
    geometry_msgs
    husarion_ugv_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME}_test_lights_manager_node
                        lights_manager_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_lights_behavior_tree
    test/test_lights_behavior_tree.cpp src/lights_manager_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_behavior_tree
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_lights_behavior_tree
    behaviortree_cpp
    behaviortree_ros2
    geometry_msgs
    husarion_ugv_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME}_test_lights_behavior_tree
                        lights_manager_parameters)

  ament_add_gtest(${PROJECT_NAME}_test_safety_manager_node
                  test/test_safety_manager_node.cpp src/safety_manager_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_safety_manager_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_safety_manager_node
    behaviortree_cpp
    behaviortree_ros2
    geometry_msgs
    husarion_ugv_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs
    tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME}_test_safety_manager_node
                        safety_manager_parameters OpenSSL::SSL)

  ament_add_gtest(
    ${PROJECT_NAME}_test_safety_behavior_tree
    test/test_safety_behavior_tree.cpp src/safety_manager_node.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_safety_behavior_tree
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_safety_behavior_tree
    behaviortree_cpp
    behaviortree_ros2
    geometry_msgs
    husarion_ugv_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs
    tf2_geometry_msgs)
  target_link_libraries(${PROJECT_NAME}_test_safety_behavior_tree
                        safety_manager_parameters OpenSSL::SSL)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_package()
