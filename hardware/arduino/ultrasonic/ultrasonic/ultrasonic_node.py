#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import UltrasonicArray
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class UltrasonicNode(Node):
    def __init__(self):
        super().__init__('ultrasonic_node')
        self.publisher = self.create_publisher(UltrasonicArray, '/ultrasonic_sensors', 10)

        # Charge la configuration
        config_path = os.path.join(
            get_package_share_directory('ultrasonic'),
            'resource',
            'config.yaml'
        )
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)

        # Ouvre le port série
        self.serial_port = serial.Serial(
            port=config['arduino']['port'],
            baudrate=config['arduino']['baudrate'],
            timeout=1
        )
        self.get_logger().info(f"Port série ouvert : {config['arduino']['port']}")

        # Timer pour lire et publier les données
        self.timer = self.create_timer(0.1, self.publish_data)

    def publish_data(self):
        msg = UltrasonicArray()
        line = self.serial_port.readline().decode('utf-8').strip()
        if line:
            try:
                distances = [float(d) for d in line.split(',')]
                if len(distances) == 4:
                    msg.distances = distances
                    self.publisher.publish(msg)
                    self.get_logger().info(f"Distances : {distances}")
            except ValueError:
                self.get_logger().warn(f"Données invalides : {line}")

    def destroy_node(self):
        self.serial_port.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
