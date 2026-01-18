#!/usr/bin/env python3
"""
Launch file pour E-WheelChAIr
Nom : ewheelchair_joystick_servo.launch.py

Ce launch file démarre uniquement :
1. Arduino Data Receiver Node (joystick + ultrasons)
2. Master Node (logique décisionnelle)
3. Servo Controller Node (commande des servos)

Objectif : piloter le fauteuil avec le joystick via Arduino et Raspberry.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # ===============================
    # 1. Arduino Data Receiver Node
    # ===============================
    # Lit les données USB de l’Arduino (joystick, ultrasons)
    # et publie des topics ROS standardisés pour le Master Node.
    arduino_data_receiver_node = Node(
        package='arduino_data_receiver',
        executable='arduino_data_receiver_node',
        name='arduino_data_receiver_node',
        output='screen',
        parameters=[{
            # Paramètres du port série (optionnel si déjà codé dans le node)
            'serial_port': '/dev/ttyACM0',
            'baudrate': 115200
        }]
    )

    # ===============================
    # 2. Master Node
    # ===============================
    # Nœud central qui :
    # - reçoit les topics du Data Receiver
    # - applique la logique décisionnelle (arrêt d’urgence, obstacles)
    # - publie les commandes de servos
    master_node = Node(
        package='master_node',
        executable='master_node',
        name='master_node',
        output='screen'
    )

    # ===============================
    # 3. Servo Controller Node
    # ===============================
    # Reçoit les commandes de servos depuis le Master Node
    # et pilote l’Arduino des servos.
    servo_controller_node = Node(
        package='servo_controller',
        executable='servo_controller_node',
        name='servo_controller_node',
        output='screen'
    )

    # ===============================
    # Retour de la LaunchDescription
    # ===============================
    return LaunchDescription([
        arduino_data_receiver_node,
        master_node,
        servo_controller_node
    ])
