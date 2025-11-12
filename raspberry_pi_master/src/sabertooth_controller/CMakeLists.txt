cmake_minimum_required(VERSION 3.8)
project(sabertooth_controller)

# Utiliser C++17 minimum
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)

# Include
include_directories(include)

# Exécutable
add_executable(sabertooth_controller_node src/sabertooth_controller.cpp)

# Dépendances
ament_target_dependencies(sabertooth_controller_node
  rclcpp
  geometry_msgs
  serial_driver
)

# Installation
install(TARGETS sabertooth_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Export
ament_export_dependencies(rclcpp geometry_msgs serial_driver)
ament_package()
