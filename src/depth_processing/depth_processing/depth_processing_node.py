#!/usr/bin/env python3
"""
Depth Processing Node
=====================
Interface USB → ROS pour la caméra de profondeur.
Le PC externe fait le traitement de profondeur.
"""

import rclpy
from rclpy.node import Node
import serial
import json

from custom_msgs.msg import ObstacleDetection


class DepthProcessingNode(Node):
    def __init__(self):
        super().__init__('depth_processing_node')

        self.declare_parameter('usb_port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('usb_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        self.publisher_ = self.create_publisher(
            ObstacleDetection,
            '/depth_detected_objects',
            10
        )

        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Depth camera USB connected on {port}")
        except Exception as e:
            self.get_logger().error(f"Erreur USB depth camera : {e}")
            self.serial = None

        self.timer = self.create_timer(0.02, self.read_usb_data)

    def read_usb_data(self):
        if self.serial is None or not self.serial.is_open:
            return

        try:
            line = self.serial.readline().decode('utf-8').strip()
            if not line:
                return

            data = json.loads(line)

            msg = ObstacleDetection()
            msg.is_human = (data.get('type') == 'human')
            msg.relative_x = float(data.get('x', 0.0))
            msg.relative_y = float(data.get('y', 0.0))
            msg.relative_distance = float(data.get('distance', 999.0))

            self.publisher_.publish(msg)

        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DepthProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
