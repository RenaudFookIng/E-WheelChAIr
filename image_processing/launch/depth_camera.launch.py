from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('image_processing'),
        'config',
        'depth_camera_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera_node',
            output='screen',
            parameters=[config]
        ),
        # Ajoutez ici le nœud de traitement depth_processing_node plus tard
    ])