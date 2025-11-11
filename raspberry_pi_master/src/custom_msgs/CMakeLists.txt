cmake_minimum_required(VERSION 3.5)
project(custom_msgs)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

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

# Export messages
rosidl_target_interfaces(${PROJECT_NAME}
  ${PROJECT_NAME}_INTERFACES
  "rosidl_typesupport_c"
  "rosidl_typesupport_cpp"
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
ament_export_dependencies(${dependencies})

# Finalization of the ament package
ament_package()
