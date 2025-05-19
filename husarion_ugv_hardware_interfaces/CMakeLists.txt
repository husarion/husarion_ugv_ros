cmake_minimum_required(VERSION 3.11)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(husarion_ugv_hardware_interfaces)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    ament_index_cpp
    controller_interface
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    hardware_interface
    imu_filter_madgwick
    husarion_ugv_msgs
    husarion_ugv_utils
    phidgets_api
    PkgConfig
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs
    tf2_geometry_msgs
    tf2_ros)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(phidgets_spatial_parameters
                           src/phidgets_spatial_parameters.yaml)

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)

add_library(
  ${PROJECT_NAME} SHARED
  src/phidget_imu_sensor/phidget_imu_sensor.cpp
  src/robot_system/gpio/gpio_controller.cpp
  src/robot_system/gpio/gpio_driver.cpp
  src/robot_system/robot_driver/canopen_manager.cpp
  src/robot_system/robot_driver/roboteq_data_converters.cpp
  src/robot_system/robot_driver/roboteq_driver.cpp
  src/robot_system/robot_driver/roboteq_error_filter.cpp
  src/robot_system/robot_driver/roboteq_robot_driver.cpp
  src/robot_system/robot_driver/lynx_robot_driver.cpp
  src/robot_system/robot_driver/panther_robot_driver.cpp
  src/robot_system/system_e_stop.cpp
  src/robot_system/system_ros_interface.cpp
  src/robot_system/lynx_system.cpp
  src/robot_system/panther_system.cpp
  src/robot_system/ugv_system.cpp
  src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP
                      PkgConfig::LIBGPIOD phidgets_spatial_parameters)

target_compile_definitions(
  ${PROJECT_NAME} PRIVATE "HUSARION_UGV_HARDWARE_INTERFACES_BUILDING_DLL")

install(
  TARGETS ${PROJECT_NAME} phidgets_spatial_parameters
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)
  find_package(husarion_ugv_utils REQUIRED)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  ament_add_gmock(${PROJECT_NAME}_test_utils test/test_utils.cpp src/utils.cpp)

  ament_add_gmock(
    ${PROJECT_NAME}_test_roboteq_error_filter
    test/unit/robot_system/robot_driver/test_roboteq_error_filter.cpp
    src/robot_system/robot_driver/roboteq_error_filter.cpp)

  ament_add_gmock(
    ${PROJECT_NAME}_test_roboteq_data_converters
    test/unit/robot_system/robot_driver/test_roboteq_data_converters.cpp
    src/robot_system/robot_driver/roboteq_data_converters.cpp src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_data_converters
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_roboteq_data_converters ament_index_cpp
    husarion_ugv_msgs husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_data_converters
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gmock(
    ${PROJECT_NAME}_test_gpiod_controller
    test/unit/robot_system/gpio/test_gpio_controller.cpp
    src/robot_system/gpio/gpio_controller.cpp
    src/robot_system/gpio/gpio_driver.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_gpiod_controller
                            ament_index_cpp husarion_ugv_utils)
  target_link_libraries(${PROJECT_NAME}_test_gpiod_controller
                        PkgConfig::LIBGPIOD)

  ament_add_gmock(
    ${PROJECT_NAME}_test_ugv_system
    test/unit/robot_system/test_ugv_system.cpp
    src/robot_system/gpio/gpio_controller.cpp
    src/robot_system/gpio/gpio_driver.cpp
    src/robot_system/system_e_stop.cpp
    src/robot_system/robot_driver/roboteq_data_converters.cpp
    src/robot_system/robot_driver/roboteq_error_filter.cpp
    src/robot_system/system_ros_interface.cpp
    src/robot_system/ugv_system.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_ugv_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_ugv_system
    ament_index_cpp
    diagnostic_updater
    hardware_interface
    rclcpp
    husarion_ugv_msgs
    husarion_ugv_utils
    std_msgs
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_ugv_system
                        PkgConfig::LIBLELY_COAPP PkgConfig::LIBGPIOD)

  ament_add_gmock(
    ${PROJECT_NAME}_test_lynx_system
    test/unit/robot_system/test_lynx_system.cpp
    src/robot_system/gpio/gpio_controller.cpp
    src/robot_system/gpio/gpio_driver.cpp
    src/robot_system/system_e_stop.cpp
    src/robot_system/robot_driver/canopen_manager.cpp
    src/robot_system/robot_driver/roboteq_robot_driver.cpp
    src/robot_system/robot_driver/roboteq_data_converters.cpp
    src/robot_system/robot_driver/roboteq_driver.cpp
    src/robot_system/robot_driver/roboteq_error_filter.cpp
    src/robot_system/robot_driver/lynx_robot_driver.cpp
    src/robot_system/system_ros_interface.cpp
    src/robot_system/lynx_system.cpp
    src/robot_system/ugv_system.cpp
    src/utils.cpp)
  set_tests_properties(${PROJECT_NAME}_test_lynx_system PROPERTIES TIMEOUT 120)
  target_include_directories(
    ${PROJECT_NAME}_test_lynx_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_lynx_system
    ament_index_cpp
    diagnostic_updater
    hardware_interface
    rclcpp
    husarion_ugv_msgs
    husarion_ugv_utils
    std_msgs
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_lynx_system
                        PkgConfig::LIBLELY_COAPP PkgConfig::LIBGPIOD)

  ament_add_gmock(
    ${PROJECT_NAME}_test_panther_system
    test/unit/robot_system/test_panther_system.cpp
    src/robot_system/gpio/gpio_controller.cpp
    src/robot_system/gpio/gpio_driver.cpp
    src/robot_system/system_e_stop.cpp
    src/robot_system/robot_driver/canopen_manager.cpp
    src/robot_system/robot_driver/roboteq_robot_driver.cpp
    src/robot_system/robot_driver/roboteq_data_converters.cpp
    src/robot_system/robot_driver/roboteq_driver.cpp
    src/robot_system/robot_driver/roboteq_error_filter.cpp
    src/robot_system/robot_driver/panther_robot_driver.cpp
    src/robot_system/system_ros_interface.cpp
    src/robot_system/panther_system.cpp
    src/robot_system/ugv_system.cpp
    src/utils.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_system PROPERTIES TIMEOUT
                                                                      120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system
    ament_index_cpp
    diagnostic_updater
    hardware_interface
    rclcpp
    husarion_ugv_msgs
    husarion_ugv_utils
    std_msgs
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_panther_system
                        PkgConfig::LIBLELY_COAPP PkgConfig::LIBGPIOD)

  # Integration tests
  option(TEST_INTEGRATION "Run integration tests" OFF)
  if(TEST_INTEGRATION)
    ament_add_gtest(
      ${PROJECT_NAME}_test_system_ros_interface
      test/integration/robot_system/test_system_ros_interface.cpp
      src/robot_system/system_ros_interface.cpp
      src/robot_system/robot_driver/roboteq_data_converters.cpp
      src/robot_system/gpio/gpio_controller.cpp
      src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_system_ros_interface
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_system_ros_interface
      diagnostic_updater
      rclcpp
      husarion_ugv_msgs
      husarion_ugv_utils
      realtime_tools
      std_srvs)
    target_link_libraries(${PROJECT_NAME}_test_system_ros_interface
                          PkgConfig::LIBLELY_COAPP PkgConfig::LIBGPIOD)

    ament_add_gmock(
      ${PROJECT_NAME}_test_phidget_imu_sensor
      test/integration/phidget_imu_sensor/test_phidget_imu_sensor.cpp)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_phidget_imu_sensor hardware_interface rclcpp
      husarion_ugv_utils husarion_ugv_msgs phidgets_api)
    target_link_libraries(${PROJECT_NAME}_test_phidget_imu_sensor
                          ${PROJECT_NAME} phidgets_spatial_parameters)

    ament_add_gtest(${PROJECT_NAME}_test_gpiod_driver
                    test/integration/robot_system/gpio/test_gpio_driver.cpp)
    ament_target_dependencies(${PROJECT_NAME}_test_gpiod_driver
                              husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_gpiod_driver ${PROJECT_NAME}
                          PkgConfig::LIBGPIOD)

    ament_add_gtest(
      ${PROJECT_NAME}_test_canopen_manager
      test/unit/robot_system/robot_driver/test_canopen_manager.cpp
      src/robot_system/robot_driver/canopen_manager.cpp src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_canopen_manager
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_canopen_manager ament_index_cpp rclcpp
      husarion_ugv_msgs husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_canopen_manager
                          PkgConfig::LIBLELY_COAPP)

    ament_add_gmock(
      ${PROJECT_NAME}_test_lynx_robot_driver
      test/unit/robot_system/robot_driver/test_lynx_robot_driver.cpp
      src/robot_system/robot_driver/canopen_manager.cpp
      src/robot_system/robot_driver/roboteq_driver.cpp
      src/robot_system/robot_driver/roboteq_data_converters.cpp
      src/robot_system/robot_driver/roboteq_robot_driver.cpp
      src/robot_system/robot_driver/lynx_robot_driver.cpp
      src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_lynx_robot_driver
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_lynx_robot_driver ament_index_cpp rclcpp
      husarion_ugv_msgs husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_lynx_robot_driver
                          PkgConfig::LIBLELY_COAPP)

    ament_add_gmock(
      ${PROJECT_NAME}_test_panther_robot_driver
      test/unit/robot_system/robot_driver/test_panther_robot_driver.cpp
      src/robot_system/robot_driver/canopen_manager.cpp
      src/robot_system/robot_driver/roboteq_driver.cpp
      src/robot_system/robot_driver/roboteq_data_converters.cpp
      src/robot_system/robot_driver/roboteq_robot_driver.cpp
      src/robot_system/robot_driver/panther_robot_driver.cpp
      src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_panther_robot_driver
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_panther_robot_driver ament_index_cpp rclcpp
      husarion_ugv_msgs husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_panther_robot_driver
                          PkgConfig::LIBLELY_COAPP)

    ament_add_gtest(
      ${PROJECT_NAME}_test_roboteq_driver
      test/unit/robot_system/robot_driver/test_roboteq_driver.cpp
      src/robot_system/robot_driver/canopen_manager.cpp
      src/robot_system/robot_driver/roboteq_driver.cpp
      src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_roboteq_driver
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_roboteq_driver ament_index_cpp rclcpp
      husarion_ugv_msgs husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_roboteq_driver
                          PkgConfig::LIBLELY_COAPP)

    ament_add_gmock(
      ${PROJECT_NAME}_test_roboteq_robot_driver
      test/unit/robot_system/robot_driver/test_roboteq_robot_driver.cpp
      src/robot_system/robot_driver/canopen_manager.cpp
      src/robot_system/robot_driver/roboteq_driver.cpp
      src/robot_system/robot_driver/roboteq_data_converters.cpp
      src/robot_system/robot_driver/roboteq_robot_driver.cpp
      src/utils.cpp)
    target_include_directories(
      ${PROJECT_NAME}_test_roboteq_robot_driver
      PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
             $<INSTALL_INTERFACE:include>)
    ament_target_dependencies(
      ${PROJECT_NAME}_test_roboteq_robot_driver ament_index_cpp rclcpp
      husarion_ugv_msgs husarion_ugv_utils)
    target_link_libraries(${PROJECT_NAME}_test_roboteq_robot_driver
                          PkgConfig::LIBLELY_COAPP)

  endif()

endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(hardware_interface
                                         husarion_ugv_hardware_interfaces.xml)

ament_package()
