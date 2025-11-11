cmake_minimum_required(VERSION 3.5)
project(sabertooth_controller)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(serial_driver REQUIRED)  

include_directories(include)

add_executable(sabertooth_controller src/sabertooth_controller.cpp)
ament_target_dependencies(sabertooth_controller rclcpp geometry_msgs serial_driver)

install(TARGETS sabertooth_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp geometry_msgs serial_driver)
ament_package()
