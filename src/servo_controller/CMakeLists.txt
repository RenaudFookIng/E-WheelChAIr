cmake_minimum_required(VERSION 3.5)
project(servo_controller)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_msgs REQUIRED)

# Servo controller node
add_executable(servo_controller_node 
  src/servo_controller/servo_controller_node.py)

# Install
install(TARGETS servo_controller_node
  DESTINATION lib/${PROJECT_NAME})

# Ament targets
ament_target_dependencies(servo_controller_node
  "rclpy"
  "custom_msgs"
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()