# src/e_wheelchair_launch/e_wheelchair_launch/launch/ewheelchair_wyes_teleop.launch.py
import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # =====================================================
    # Arduino Bridge Node
    # =====================================================
    arduino_bridge_node = Node(
        package='arduino_bridge',
        executable='arduino_bridge_node',
        name='arduino_bridge_node',
        output='screen',
        emulate_tty=True
    )

    # =====================================================
    # Master Node
    # =====================================================
    master_node = Node(
        package='master_node',
        executable='master_node',
        name='master_node',
        output='screen',
        emulate_tty=True
    )

    """# =====================================================
    # Wyes Teleop Node
    # =====================================================
    wyes_teleop_node = Node(
        package='wyes_teleop',
        executable='wyes_teleop',
        name='wyes_teleop',
        output='screen',
        emulate_tty=True
    )"""

    return LaunchDescription([
        arduino_bridge_node,
        master_node
    ])
