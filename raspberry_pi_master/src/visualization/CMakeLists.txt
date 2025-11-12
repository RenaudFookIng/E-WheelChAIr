cmake_minimum_required(VERSION 3.8)
project(sabertooth_controller)

# Trouver les dépendances
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(serial_driver REQUIRED)

# Ajouter l'exécutable
add_executable(sabertooth_node src/sabertooth_controller.cc)

# Lier les bibliothèques
ament_target_dependencies(sabertooth_node rclcpp serial_driver)

# Installer
install(TARGETS sabertooth_node DESTINATION lib/${PROJECT_NAME})

# Exporter les dépendances
ament_export_dependencies(rclcpp serial_driver)

ament_package()
