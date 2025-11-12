#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import numpy as np

class MotorSpeedCalculator:
    def __init__(self, wheel_radius=0.1, gear_ratio=10.0, max_voltage=12.0, max_rpm=100.0):  # (a corriger)
        self.wheel_radius = wheel_radius
        self.gear_ratio = gear_ratio
        self.max_voltage = max_voltage
        self.max_rpm = max_rpm

    def calculate_speed(self, voltage):
        rpm = (voltage / self.max_voltage) * self.max_rpm
        return (rpm * 2 * np.pi / 60) * self.wheel_radius  # m/s

class MotorSpeedNode(Node):
    def __init__(self):
        super().__init__('motor_speed_node')
        self.calculator = MotorSpeedCalculator()
        self.publisher = self.create_publisher(Float32, '/motor_speed', 10)
        self.subscription = self.create_subscription(
            Float32,
            '/motor_voltage',
            self.voltage_callback,
            10)
        self.timer = self.create_timer(0.1, self.publish_speed)

    def voltage_callback(self, msg):
        self.current_voltage = msg.data

    def publish_speed(self):
        if hasattr(self, 'current_voltage'):
            speed = self.calculator.calculate_speed(self.current_voltage)
            self.publisher.publish(Float32(data=speed))

def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
