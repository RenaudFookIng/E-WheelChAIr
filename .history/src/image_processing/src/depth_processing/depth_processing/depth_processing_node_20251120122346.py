# transforme une information d'un ordinateur a un topic ros2

import rclpy
from rclpy.node import Node
import serial


class DepthProcessingNode(Node):
    def __init__(self):
        super().__init__('depth_processing_node')
        self.publisher_ = self.create_publisher(String, 'depth_data', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Configuration du port série
        self.ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
        self.ser.flush()

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            depth_data = self.ser.readline().decode('utf-8').rstrip()
            msg = String()
            msg.data = depth_data
            self.publisher_.publish(msg)
            self.get_logger().info(f'Published depth data: {msg.data}')