cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_cmake_python
    diagnostic_updater
    realtime_tools
    rclcpp
    std_msgs
    yaml-cpp)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(${PROJECT_NAME}_test_common_utilities
                  test/test_common_utilities.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_common_utilities
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_diagnostics test/test_diagnostics.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_diagnostics
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_diagnostics diagnostic_updater)

  ament_add_gtest(${PROJECT_NAME}_test_moving_average
                  test/test_moving_average.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_moving_average
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_test_utils test/test_test_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_test_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)

  ament_add_gtest(${PROJECT_NAME}_test_ros_utils test/test_ros_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_ros_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_ros_utils std_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_ros_test_utils
                  test/test_ros_test_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_ros_test_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_ros_test_utils rclcpp std_msgs)

  ament_add_gtest(${PROJECT_NAME}_test_yaml_utils test/test_yaml_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_yaml_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
  target_link_libraries(${PROJECT_NAME}_test_yaml_utils yaml-cpp)

  ament_add_gtest(${PROJECT_NAME}_test_networking_utils
                  test/test_networking_utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_networking_utils
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
           $<INSTALL_INTERFACE:include>)
endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})

ament_python_install_package(${PROJECT_NAME})

ament_package()
