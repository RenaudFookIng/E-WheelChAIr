#!/usr/bin/env python3
"""
Launch file for E-WheelChAIr
============================
Lance les 2 nodes essentiels pour le pilotage via joystick :
- ArduinoBridgeNode : Noeud de communication entre Arduino et Raspberry
- MasterNode : décision de commande et logique sécurité
"""

import os

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # =====================================================
    # Node Arduino Data Receiver
    # =====================================================
    arduino_bridge_node = Node(
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
    # Return LaunchDescription
    # =====================================================
    return LaunchDescription([
        arduino_bridge_node,
        master_node
    ])
