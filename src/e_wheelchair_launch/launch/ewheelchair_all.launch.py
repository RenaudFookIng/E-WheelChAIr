from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch.substitutions import FindExecutable

def generate_launch_description():
    return LaunchDescription([
        # Sabertooth Controller
        Node(
            package='sabertooth_controller',
            executable='sabertooth_controller_node',
            name='sabertooth_controller',
            output='screen'
        ),

        # Motor Speed Calculator
        Node(
            package='motor_speed_calculator',
            executable='motor_speed_calculator_node',
            name='motor_speed_calculator',
            output='screen'
        ),

        # Depth Camera Driver
        Node(
            package='depth_camera_driver',
            executable='depth_camera_node',
            name='depth_camera_driver',
            output='screen'
        ),

        # Wide Camera Driver
        Node(
            package='wide_camera_driver',
            executable='wide_camera_node',
            name='wide_camera_driver',
            output='screen'
        ),

        # Lidar
        Node(
            package='lidar',
            executable='lidar_node',
            name='lidar',
            output='screen'
        ),

        # Visualization
        Node(
            package='visualization',
            executable='visualization_node',
            name='visualization',
            output='screen'
        ),
    ])
