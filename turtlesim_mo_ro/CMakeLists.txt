cmake_minimum_required(VERSION 3.5)
project(turtlesim_mo_ro)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(turtlesim_mo_ro src/turtlesim_mo_ro.cpp)

ament_target_dependencies(turtlesim_mo_ro rclcpp std_msgs turtlesim geometry_msgs)

install(TARGETS
  turtlesim_mo_ro
  DESTINATION lib/${PROJECT_NAME})

ament_package()

