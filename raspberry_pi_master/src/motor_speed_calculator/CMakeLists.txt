cmake_minimum_required(VERSION 3.8)
project(motor_speed_calculator)

# Options de compilation pour le C++ (si jamais vous en ajoutez)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Trouver les dépendances ROS2
find_package(ament_cmake REQUIRED)

# Installer le module Python (obligatoire pour que ROS2 trouve votre package)
install(
  DIRECTORY
    motor_speed_calculator
  DESTINATION lib/python3.8/site-packages  # Remplacez 3.8 par votre version de Python
)

# Désactiver les vérifications de linting (optionnel)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)  # Désactive la vérification des copyrights
  set(ament_cmake_cpplint_FOUND TRUE)   # Désactive cpplint (inutile pour Python)
  ament_lint_auto_find_test_dependencies()
endif()

# Déclarer le package ROS2
ament_package()
