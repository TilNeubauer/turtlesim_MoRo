cmake_minimum_required(VERSION 3.5)
project(turtlesim_automata)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(geometry_msgs REQUIRED)  # Fügen Sie diese Zeile hinzu

add_executable(turtlesim_automata src/turtlesim_automata.cpp)

ament_target_dependencies(turtlesim_automata rclcpp std_msgs turtlesim geometry_msgs)  # Fügen Sie geometry_msgs hinzu

install(TARGETS
  turtlesim_automata
  DESTINATION lib/${PROJECT_NAME})

ament_package()

