cmake_minimum_required(VERSION 3.8)
project(custom_msgs)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Generate message interfaces files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UltrasonicArray.msg"
  "msg/Joystick.msg"
  "msg/EmergencyData.msg"
  "msg/ObstacleDetection.msg"
)

# Dependencies for message generation
set(dependencies
rclcpp  
std_msgs
)

# Installation
install(
  DIRECTORY
   msg
  DESTINATION share/${PROJECT_NAME}
)

# Runtime dependencies
ament_export_dependencies(rosidl_default_runtime)

# Export dependencies for packages that use custom_msgs
ament_export_dependencies(include)

# Finalization of the ament package
ament_package()
