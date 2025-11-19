#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import Joystick
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joystick_node')
        self.publisher = self.create_publisher(Joystick, '/joystick_input', 10)

        # Charge la configuration
        config_path = os.path.join(
            get_package_share_directory('joystick'),
            'resource',
            'config.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Ouvre le port série (si le joystick est connecté via Arduino)
        self.serial_port = serial.Serial(
            port=config['arduino']['port'],
            baudrate=config['arduino']['baudrate'],
            timeout=1
        )
        self.get_logger().info(f"Port série ouvert : {config['arduino']['port']}")

        # Timer pour lire et publier les données
        self.timer = self.create_timer(0.05, self.publish_data)  # 20 Hz

    def publish_data(self):
        msg = Joystick()
        line = self.serial_port.readline().decode('utf-8').strip()
        if line:
            try:
                x_axis, y_axis = map(float, line.split(','))
                msg.x_axis = x_axis
                msg.y_axis = y_axis
                self.publisher.publish(msg)
                self.get_logger().info(f"Joystick : x={x_axis}, y={y_axis}")
            except ValueError:
                self.get_logger().warn(f"Données invalides : {line}")

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JoystickNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
