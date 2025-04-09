cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    hardware_interface
    gz-sim8
    gz-msgs10
    gz-plugin2
    gz-transport13
    gz_ros2_control
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
    std_srvs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

include_directories(include ${Qt5Core_INCLUDE_DIRS} ${Qt5Qml_INCLUDE_DIRS}
                    ${Qt5Quick_INCLUDE_DIRS} ${Qt5QuickControls2_INCLUDE_DIRS})

add_library(estop_system SHARED src/estop_system.cpp)
ament_target_dependencies(
  estop_system
  hardware_interface
  gz_ros2_control
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs)
target_link_libraries(estop_system gz-sim8)

set(CMAKE_AUTOMOC ON)
qt5_add_resources(resources_rcc src/gui/EStop.qrc)

add_library(EStop SHARED include/husarion_ugv_gazebo/gui/e_stop.hpp
                         src/gui/e_stop.cpp ${resources_rcc})
ament_target_dependencies(EStop rclcpp std_msgs std_srvs)
target_link_libraries(
  EStop
  gz-sim8
  gz-plugin2
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES})

add_library(LEDStrip SHARED src/led_strip.cpp)
ament_target_dependencies(LEDStrip realtime_tools)
target_link_libraries(LEDStrip gz-sim8 gz-msgs10 gz-plugin2 gz-transport13)

install(
  TARGETS estop_system EStop LEDStrip
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/hooks/setup_envs.sh.in")

ament_export_dependencies(${PACKAGE_DEPENDENCIES})
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(gz_ros2_control plugins.xml)

ament_package()
