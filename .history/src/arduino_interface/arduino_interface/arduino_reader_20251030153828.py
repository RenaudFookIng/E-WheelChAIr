#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial

class ArduinoReader(Node):
    def __init__(self):
        super().__init__('arduino_reader')
        self.publisher = self.create_publisher(String, 'arduino_data', 10)
        self.serial_port = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

    def read_data(self):
        while rclpy.ok():
            data = self.serial_port.readline().decode('utf-8').strip()
            if data:
                self.publisher.publish(String(data=data))
                self.get_logger().info(f"Données Arduino : {data}")

def main(args=None):
    rclpy.init(args=args)
    node = ArduinoReader()
    node.read_data()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
