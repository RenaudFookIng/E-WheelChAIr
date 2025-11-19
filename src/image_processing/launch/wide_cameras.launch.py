from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('image_processing'),
        'config',
        'wide_camera_params.yaml'
    )

    with open(config_path, 'r') as f:
        params = yaml.safe_load(f)

    camera_nodes = []
    for camera in params['wide_cameras']:
        camera_nodes.append(
            Node(
                package='usb_cam',
                executable='usb_cam_node_exe',
                name=camera['camera_name'],
                output='screen',
                parameters=[
                    {'video_device': camera['video_device']},
                    {'image_width': camera['image_width']},
                    {'image_height': camera['image_height']},
                    {'pixel_format': camera['pixel_format']},
                    {'framerate': camera['framerate']},
                    {'camera_info_url': 'package://image_processing/config/camera_info.yaml'}
                ],
                remappings=[
                    ('image_raw', f"{camera['camera_name']}/image_raw")
                ]
            )
        )

    # nœud de traitement wide_processing_node plus tard
    camera_nodes.append(
        Node(
            package='wide_processing',
            executable='wide_processing_node',
            name='wide_processing_node',
            output='screen',
            parameters=[
                {'use_sim_time': True}
            ]
        )
    )

    return LaunchDescription(camera_nodes)