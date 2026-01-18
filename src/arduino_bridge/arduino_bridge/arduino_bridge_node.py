#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import threading

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int16MultiArray


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # Publishers
        self.joy_pub = self.create_publisher(Twist, '/joystick/raw', 10)
        self.us_pub = self.create_publisher(Float32MultiArray, '/ultrasonic/data', 10)

        # Subscriber (commandes servo)
        self.servo_sub = self.create_subscription(
            Int16MultiArray,
            '/servo/command',
            self.servo_callback,
            10
        )

        # Serial
        self.serial = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)
        self.get_logger().info("Arduino connecté sur /dev/ttyACM0")

        # Thread lecture série
        self.thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.thread.start()

    # =============================
    #      Arduino → ROS
    # =============================
    def read_serial_loop(self):
        buffer = ""
        while rclpy.ok():
            try:
                buffer += self.serial.read().decode('utf-8', errors='ignore')
                if '\n' not in buffer:
                    continue

                line, buffer = buffer.split('\n', 1)
                data = json.loads(line)

                if data.get("type") == "sensor_data":
                    self.handle_sensor_data(data)

            except Exception as e:
                self.get_logger().warn(f"Serial error: {e}")

    def handle_sensor_data(self, data):
        # Joystick
        joy = Twist()
        joy.linear.x = float(data["joystick"]["y"])
        joy.angular.z = float(data["joystick"]["x"])
        self.joy_pub.publish(joy)

        # Ultrasons
        us = Float32MultiArray()
        us.data = [d / 100.0 for d in data["ultrasonic"]]  # cm → m
        self.us_pub.publish(us)

    # =============================
    #      ROS → Arduino
    # =============================
    def servo_callback(self, msg: Int16MultiArray):
        if len(msg.data) < 2:
            return

        cmd = {
            "servo_command": {
                "x_angle": int(msg.data[0]),
                "y_angle": int(msg.data[1])
            }
        }
        self.serial.write((json.dumps(cmd) + "\n").encode('utf-8'))


def main():
    rclpy.init()
    node = ArduinoBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
