cmake_minimum_required(VERSION 3.5)
project(teleop_joystick)

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

# Teleop joystick node
add_executable(teleop_joystick_node 
  src/teleop_joystick/teleop_joystick_node.py)

# Install
install(TARGETS teleop_joystick_node
  DESTINATION lib/${PROJECT_NAME})

# Ament targets
ament_target_dependencies(teleop_joystick_node
  "rclpy"
  "custom_msgs"
)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/)

ament_package()