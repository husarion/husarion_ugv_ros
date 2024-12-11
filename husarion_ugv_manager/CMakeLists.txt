cmake_minimum_required(VERSION 3.8)
project(husarion_ugv_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    behaviortree_cpp
    behaviortree_ros2
    generate_parameter_library
    libssh
    panther_msgs
    husarion_ugv_utils
    rclcpp
    rclcpp_action
    sensor_msgs
    std_srvs
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

add_library(shutdown_single_host_bt_node SHARED
            src/plugins/action/shutdown_single_host_node.cpp)
target_link_libraries(shutdown_single_host_bt_node ssh)
list(APPEND plugin_libs shutdown_single_host_bt_node)

add_library(shutdown_hosts_from_file_bt_node SHARED
            src/plugins/action/shutdown_hosts_from_file_node.cpp)
target_link_libraries(shutdown_hosts_from_file_bt_node ssh yaml-cpp)
list(APPEND plugin_libs shutdown_hosts_from_file_bt_node)

add_library(tick_after_timeout_bt_node SHARED
            src/plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_bt_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
  ament_target_dependencies(${bt_plugin} ${PACKAGE_DEPENDENCIES})
endforeach()

add_executable(
  safety_manager_node src/safety_manager_node_main.cpp
                      src/safety_manager_node.cpp src/behavior_tree_manager.cpp)
ament_target_dependencies(
  safety_manager_node
  behaviortree_ros2
  panther_msgs
  husarion_ugv_utils
  rclcpp
  sensor_msgs
  std_msgs)

generate_parameter_library(safety_manager_parameters
                           config/safety_manager_parameters.yaml)
target_link_libraries(safety_manager_node ${plugin_libs}
                      safety_manager_parameters)

add_executable(
  lights_manager_node src/lights_manager_node_main.cpp
                      src/lights_manager_node.cpp src/behavior_tree_manager.cpp)
ament_target_dependencies(
  lights_manager_node
  behaviortree_ros2
  panther_msgs
  husarion_ugv_utils
  rclcpp
  sensor_msgs
  std_msgs)

generate_parameter_library(lights_manager_parameters
                           config/lights_manager_parameters.yaml)
target_link_libraries(lights_manager_node ${plugin_libs}
                      lights_manager_parameters)

install(TARGETS ${plugin_libs} DESTINATION lib)

install(TARGETS safety_manager_node lights_manager_node
        DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY behavior_trees config launch
        DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(LIBSSH REQUIRED libssh)

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
    ${PROJECT_NAME}_test_signal_shutdown_node
    test/plugins/action/test_signal_shutdown_node.cpp
    src/plugins/action/signal_shutdown_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_signal_shutdown_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_shutdown_single_host_node
    test/plugins/action/test_shutdown_single_host_node.cpp
    src/plugins/action/shutdown_single_host_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_single_host_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_shutdown_hosts_from_file_node
    test/plugins/action/test_shutdown_hosts_from_file_node.cpp
    src/plugins/action/shutdown_hosts_from_file_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_from_file_node)

  ament_add_gtest(
    ${PROJECT_NAME}_test_tick_after_timeout_node
    test/plugins/decorator/test_tick_after_timeout_node.cpp
    src/plugins/decorator/tick_after_timeout_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_tick_after_timeout_node)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_host
                  test/plugins/test_shutdown_host.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_host)

  ament_add_gtest(${PROJECT_NAME}_test_shutdown_hosts_node
                  test/plugins/test_shutdown_hosts_node.cpp)
  list(APPEND plugin_tests ${PROJECT_NAME}_test_shutdown_hosts_node)

  foreach(bt_node_test ${plugin_tests})
    target_include_directories(
      ${bt_node_test}
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>
      PRIVATE ${LIBSSH_INCLUDE_DIRS})

    ament_target_dependencies(${bt_node_test} ${PACKAGE_DEPENDENCIES})
    target_link_libraries(${bt_node_test} ${LIBSSH_LIBRARIES} yaml-cpp)
  endforeach()

  ament_add_gtest(${PROJECT_NAME}_test_behavior_tree_utils
                  test/test_behavior_tree_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_behavior_tree_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_behavior_tree_utils behaviortree_cpp behaviortree_ros2
    husarion_ugv_utils)

  ament_add_gtest(
    ${PROJECT_NAME}_test_behavior_tree_manager
    test/test_behavior_tree_manager.cpp src/behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_behavior_tree_manager
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_behavior_tree_manager
                            behaviortree_cpp husarion_ugv_utils)

  ament_add_gtest(
    ${PROJECT_NAME}_test_lights_manager_node test/test_lights_manager_node.cpp
    src/lights_manager_node.cpp src/behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_manager_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_lights_manager_node
    behaviortree_cpp
    behaviortree_ros2
    panther_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs)
  target_link_libraries(${PROJECT_NAME}_test_lights_manager_node
                        lights_manager_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_lights_behavior_tree
    test/test_lights_behavior_tree.cpp src/lights_manager_node.cpp
    src/behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_lights_behavior_tree
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_lights_behavior_tree
    behaviortree_cpp
    behaviortree_ros2
    panther_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs)
  target_link_libraries(${PROJECT_NAME}_test_lights_behavior_tree
                        lights_manager_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_safety_manager_node test/test_safety_manager_node.cpp
    src/safety_manager_node.cpp src/behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_safety_manager_node
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_safety_manager_node
    behaviortree_cpp
    behaviortree_ros2
    panther_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs)
  target_link_libraries(${PROJECT_NAME}_test_safety_manager_node
                        safety_manager_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_safety_behavior_tree
    test/test_safety_behavior_tree.cpp src/safety_manager_node.cpp
    src/behavior_tree_manager.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_safety_behavior_tree
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_safety_behavior_tree
    behaviortree_cpp
    behaviortree_ros2
    panther_msgs
    husarion_ugv_utils
    rclcpp
    sensor_msgs
    std_msgs
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_safety_behavior_tree
                        safety_manager_parameters)
endif()

ament_package()
