cmake_minimum_required(VERSION 3.8)
project(master_node)

# ===========================
# Dépendances
# ===========================
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(custom_msgs REQUIRED)

# ===========================
# Installation Python
# ===========================
ament_python_install_package(${PROJECT_NAME})

# ===========================
# Installation des scripts
# ===========================
install(
  PROGRAMS
    master_node/master_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# ===========================
# Installation des fichiers de config
# ===========================
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# ===========================
# Finalisation
# ===========================
ament_package()
