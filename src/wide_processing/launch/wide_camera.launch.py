from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='wide_processing',
            executable='wide_processing_node',
            name='wide_processing_node',
            output='screen'
        )
    ])
