cmake_minimum_required(VERSION 3.10.2)
project(husarion_ugv_localization)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
