cmake_minimum_required(VERSION 3.11)
project(husarion_ugv_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  "msg/ChargingStatus.msg"
  "msg/DriverState.msg"
  "msg/DriverStateNamed.msg"
  "msg/FaultFlag.msg"
  "msg/IOState.msg"
  "msg/LEDAnimation.msg"
  "msg/LEDAnimationQueue.msg"
  "msg/LEDImageAnimation.msg"
  "msg/RobotDriverState.msg"
  "msg/RuntimeError.msg"
  "msg/ScriptFlag.msg"
  "msg/SystemStatus.msg"
  "srv/SetLEDAnimation.srv"
  "srv/SetLEDBrightness.srv"
  "srv/SetLEDImageAnimation.srv"
  DEPENDENCIES
  std_msgs)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
