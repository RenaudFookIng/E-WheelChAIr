cmake_minimum_required(VERSION 3.8)
project(wide_processing)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
  DIRECTORY src/
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
