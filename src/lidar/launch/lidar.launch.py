from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar'),
        'resource',
        'config.yaml'
    )

    return LaunchDescription([
        Node(
            package='sllidar_ros2',  # ou ldlidar_stl_ros2
            executable='sllidar_node',
            name='sllidar_node',
            output='screen',
            parameters=[config]
        )
    ])
