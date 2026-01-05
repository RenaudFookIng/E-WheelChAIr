#!/usr/bin/env python3
"""
Minimal Launch file for E-WheelChAIr Teleoperation System
Launches only the essential nodes for joystick teleoperation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Minimal launch file for teleoperation - joystick and servo control only
    """
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'arduino_joystick_port',
            default_value='/dev/ttyACM0',
            description='Serial port for Arduino joystick'
        ),
        
        DeclareLaunchArgument(
            'arduino_servo_port',
            default_value='/dev/ttyACM1',
            description='Serial port for Arduino servo controller'
        ),
        
        # Teleop Joystick Node - Reads from Arduino joystick and publishes commands
        Node(
            package='teleop_joystick',
            executable='teleop_joystick_node',
            name='teleop_joystick',
            output='screen',
            parameters=[
                {
                    'arduino.port': LaunchConfiguration('arduino_joystick_port'),
                    'arduino.baudrate': 115200,
                    'arduino.timeout': 1
                },
                os.path.join(
                    get_package_share_directory('teleop_joystick'),
                    'config',
                    'joystick_config.yaml'
                )
            ]
        ),
        
        # Servo Controller Node - Receives commands and controls servos via Arduino
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