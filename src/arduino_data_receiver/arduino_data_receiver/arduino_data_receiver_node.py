#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Range

import serial
import json
import threading


class ArduinoDataReceiverNode(Node):
    def __init__(self):
        super().__init__('arduino_data_receiver_node')

        # Publishers
        self.joystick_pub = self.create_publisher(
            Float32MultiArray,
            '/joystick/data',
            10
        )

        self.ultrasonic_pub = self.create_publisher(
            Range,
            '/ultrasonic/data',
            10
        )

        # Serial port
        self.serial_port = serial.Serial(
            '/dev/ttyACM0',
            115200,
            timeout=1
        )

        self.get_logger().info("Arduino Data Receiver Node démarré.")

        # Thread de lecture série
        self.serial_thread = threading.Thread(
            target=self.read_serial_loop,
            daemon=True
        )
        self.serial_thread.start()

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                line = self.serial_port.readline().decode('utf-8').strip()
                if not line:
                    continue

                data = json.loads(line)
                msg_type = data.get("type")

                if msg_type == "joystick":
                    self.handle_joystick(data)

                elif msg_type == "ultrasonic":
                    self.handle_ultrasonic(data)

            except json.JSONDecodeError:
                self.get_logger().warn("JSON invalide reçu")
            except Exception as e:
                self.get_logger().error(f"Erreur série: {e}")

    def handle_joystick(self, data):
        try:
            msg = Float32MultiArray()
            msg.data = [
                float(data["x"]),
                float(data["y"])
            ]
            self.joystick_pub.publish(msg)
        except KeyError:
            self.get_logger().warn("Joystick: données manquantes")

    def handle_ultrasonic(self, data):
        try:
            for i, distance_cm in enumerate(data["sensors"]):
                msg = Range()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = f"ultrasonic_{i+1}"
                msg.range = float(distance_cm) / 100.0
                self.ultrasonic_pub.publish(msg)
        except KeyError:
            self.get_logger().warn("Ultrasonic: données manquantes")


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDataReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
