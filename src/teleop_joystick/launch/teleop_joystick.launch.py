#!/usr/bin/env python3
"""
Launch file for E-WheelChAIr Teleoperation System
Launches the teleop joystick node and servo controller for direct joystick control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for teleoperation system with joystick and servo control
    """
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
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
            ],
            remappings=[
                ('/joystick_data', '/joystick_data'),
                ('/servo_commands', '/servo_commands')
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
            ],
            remappings=[
                ('/servo_commands', '/servo_commands'),
                ('/servo_status', '/servo_status'),
                ('/ultrasonic_data', '/ultrasonic_data')
            ]
        ),
        
        # Optional: Visualization Node for monitoring
        Node(
            package='visualization',
            executable='real_time_plot',
            name='visualization',
            output='screen',
            parameters=[
                {
                    'topics': ['/joystick_data', '/servo_status', '/ultrasonic_data']
                }
            ]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()