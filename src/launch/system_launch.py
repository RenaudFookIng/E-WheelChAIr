from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arduino_interface',
            executable='arduino_reader',
            name='arduino_reader'
        ),
        Node(
            package='sabertooth_controller',
            executable='sabertooth_node',
            name='sabertooth_controller'
        ),
        Node(
            package='yolo_detector',
            executable='yolo_node',
            name='yolo_detector'
        ),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{'video_device': '/dev/video0'}]
        )
    ])
