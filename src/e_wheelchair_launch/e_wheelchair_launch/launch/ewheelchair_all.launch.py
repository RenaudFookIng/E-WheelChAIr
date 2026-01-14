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
        
        # Teleop Joystick Node
        Node(
            package='teleop_joystick',
            executable='teleop_joystick_node',
            name='teleop_joystick',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('teleop_joystick'),
                    'config',
                    'joystick_config.yaml'
                )
            ]
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
        
        # LiDAR Node
        Node(
            package='lidar',
            executable='lidar_node',
            name='lidar',
            output='screen'
        ),
        
        # Image Processing Nodes (commented out by default - uncomment when needed)
        # Node(
        #     package='image_processing',
        #     executable='depth_camera_driver',
        #     name='depth_camera_driver',
        #     output='screen'
        # ),
        # Node(
        #     package='image_processing',
        #     executable='wide_camera_driver',
        #     name='wide_camera_driver',
        #     output='screen'
        # ),
        
        # Visualization Node
        Node(
            package='visualization',
            executable='real_time_plot',
            name='visualization',
            output='screen'
        )
    ])

if __name__ == '__main__':
    generate_launch_description()