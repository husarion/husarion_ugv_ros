cmake_minimum_required(VERSION 3.11)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(husarion_ugv_diagnostics)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    diagnostic_msgs
    diagnostic_updater
    generate_parameter_library
    husarion_ugv_msgs
    husarion_ugv_utils
    PkgConfig
    rclcpp
    std_msgs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

set(CPPUPROFILE_PREFIX ${CMAKE_BINARY_DIR}/ep_cppuprofile/src/ep_cppuprofile)
set(ENV{PKG_CONFIG_PATH} "${CPPUPROFILE_PREFIX}/lib:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(CPPUPROFILE REQUIRED IMPORTED_TARGET cppuprofile)

generate_parameter_library(system_monitor_parameters
                           src/system_monitor_parameters.yaml)

add_executable(system_monitor_node src/main.cpp src/system_monitor_node.cpp)
target_include_directories(system_monitor_node
                           PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
ament_target_dependencies(system_monitor_node ${PACKAGE_DEPENDENCIES})
target_link_libraries(system_monitor_node system_monitor_parameters
                      PkgConfig::CPPUPROFILE)

install(TARGETS system_monitor_node DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ros_testing REQUIRED)

  # Unit tests
  ament_add_gmock(
    ${PROJECT_NAME}_test_system_monitor test/unit/test_system_monitor_node.cpp
    src/system_monitor_node.cpp)
  target_include_directories(${PROJECT_NAME}_test_system_monitor
                             PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
  ament_target_dependencies(${PROJECT_NAME}_test_system_monitor
                            ament_cmake_gtest ${PACKAGE_DEPENDENCIES})
  target_link_libraries(${PROJECT_NAME}_test_system_monitor
                        system_monitor_parameters PkgConfig::CPPUPROFILE)

  ament_add_gmock(${PROJECT_NAME}_test_filesystem test/unit/test_filesystem.cpp)
  target_include_directories(${PROJECT_NAME}_test_filesystem
                             PUBLIC ${CMAKE_INSTALL_PREFIX}/include)
  ament_target_dependencies(${PROJECT_NAME}_test_filesystem ament_cmake_gtest
                            ${PACKAGE_DEPENDENCIES})

  # Integration tests
  option(TEST_INTEGRATION "Run integration tests" OFF)
  if(TEST_INTEGRATION)
    add_ros_test(test/integration/system_monitor_node.test.py)
  endif()

endif()

ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
