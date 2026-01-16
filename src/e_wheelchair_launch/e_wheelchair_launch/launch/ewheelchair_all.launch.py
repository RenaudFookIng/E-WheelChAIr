from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    """
    Launch file for complete E-WheelChAIr system
    """
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),
        
        # Wyes Teleop Node
        Node(
            package='wyes_teleop',
            executable='wyes_teleop_node',
            name='wyes_teleop',
            output='screen'
        ),
        
        # Servo Controller Node
        Node(
            package='servo_controller',
            executable='servo_controller_node',
            name='servo_controller',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('servo_controller'),
                    'config',
                    'servo_config.yaml'
                )
            ]
        ),
        
        # Master Node (fusion sensorielle et contrôle)
        Node(
            package='master_node',
            executable='master_node',
            name='master_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('master_node'),
                    'config',
                    'master_config.yaml'
                )
            ]
        ),
        
        # Arduino Data Receiver Node
        Node(
            package='arduino_data_receiver',
            executable='arduino_data_receiver_node',
            name='arduino_data_receiver',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('arduino_data_receiver'),
                    'resource',
                    'arduino_data_receiver'
                )
            ]
        ),
        
        # Depth Processing Node
        Node(
            package='depth_processing',
            executable='depth_processing_node',
            name='depth_processing',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('depth_processing'),
                    'resource',
                    'depth_processing'
                )
            ]
        ),

        # Wide Processing Node
        Node(
            package='wide_processing',
            executable='wide_processing_node',
            name='wide_processing',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('wide_processing'),
                    'resource',
                    'wide_processing'
                )
            ]
        ),
        

    ])

if __name__ == '__main__':
    generate_launch_description()