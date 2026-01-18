#!/usr/bin/env python3
"""
Launch file for E-WheelChAIr
============================
Lance les trois nodes essentiels pour le pilotage via joystick :
- ArduinoDataReceiverNode : lecture joystick + capteurs Arduino
- MasterNode : décision de commande et logique sécurité
- ServoControllerNode : envoi des commandes aux servos
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # =====================================================
    # Node Arduino Data Receiver
    # =====================================================
    arduino_receiver_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        output='screen',
        emulate_tty=True
    )

    # =====================================================
    # Node Master
    # =====================================================
    master_node = Node(
        package='master_node',
        executable='master_node',
        name='master_node',
        output='screen',
        emulate_tty=True
    )

    # =====================================================
    # Node Servo Controller
    # =====================================================
    servo_controller_node = Node(
        package='servo_controller',
        executable='servo_controller_node',
        name='servo_controller_node',
        output='screen',
        emulate_tty=True
    )

    # =====================================================
    # Return LaunchDescription
    # =====================================================
    return LaunchDescription([
        arduino_receiver_node,
        master_node,
        servo_controller_node
    ])
