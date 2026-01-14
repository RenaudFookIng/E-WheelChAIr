#!/usr/bin/env python3
"""
Launch file for Servo Controller Node
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'arduino_servo_port',
            default_value='/dev/ttyACM1',
            description='Serial port for Arduino servo controller'
        ),
        
        # Servo Controller Node
        Node(
            package='servo_controller',
            executable='servo_controller_node',
            name='servo_controller',
            output='screen',
            parameters=[
                {
                    'servo.port': LaunchConfiguration('arduino_servo_port'),
                    'servo.baudrate': 115200,
                    'servo.neutral_x': 90,
                    'servo.neutral_y': 90,
                    'servo.custom_neutral_x': 90,
                    'servo.custom_neutral_y': 85,
                    'servo.amplitude': 15,
                    'servo.use_custom_mapping': True
                },
                os.path.join(
                    get_package_share_directory('servo_controller'),
                    'config',
                    'servo_config.yaml'
                )
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()