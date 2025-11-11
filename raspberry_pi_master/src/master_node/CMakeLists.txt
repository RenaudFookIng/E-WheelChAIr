cmake_minimum_required(VERSION 3.5)
project(master_node)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(custom_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(master_node src/master_node.cpp)
ament_target_dependencies(master_node rclcpp custom_msgs geometry_msgs)

install(TARGETS master_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rclcpp custom_msgs geometry_msgs)
ament_package()
