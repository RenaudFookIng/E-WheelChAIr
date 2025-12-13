cmake_minimum_required(VERSION 3.8)
project(visualization)

# Trouver les dépendances ROS2
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

# Installer le module Python
install(
  DIRECTORY
    visualization
  DESTINATION lib/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages/
)

# Déclarer le package ROS2
ament_package()
