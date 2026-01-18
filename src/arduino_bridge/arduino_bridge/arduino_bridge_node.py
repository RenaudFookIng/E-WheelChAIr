#!/usr/bin/env python3
"""
Arduino Bridge Node
===================
Pont bidirectionnel entre Arduino (USB/JSON) et ROS 2.

- Arduino -> ROS : joystick, ultrasons, emergency
- ROS -> Arduino : commandes servo finales (ServoCommand)

AUCUNE logique de décision ici.
"""

import rclpy
from rclpy.node import Node

import serial
import json
import threading

# ===============================
#        MESSAGES CUSTOM
# ===============================
from custom_msgs.msg import (
    Joystick,
    UltrasonicArray,
    EmergencyData,
    ServoCommand
)


class ArduinoBridgeNode(Node):
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # =====================================================
        #                    PUBLISHERS
        # =====================================================
        self.joystick_pub = self.create_publisher(
            Joystick,
            '/joystick/data',
            10
        )

        self.ultrasonic_pub = self.create_publisher(
            UltrasonicArray,
            '/ultrasonic/data',
            10
        )

        self.emergency_pub = self.create_publisher(
            EmergencyData,
            '/emergency_data',
            10
        )

        # =====================================================
        #                    SUBSCRIBERS
        # =====================================================
        self.create_subscription(
            ServoCommand,
            '/servo_commands',
            self.servo_command_callback,
            10
        )

        # =====================================================
        #                 CONNEXION SÉRIE
        # =====================================================
        self.serial_port = serial.Serial(
            port='/dev/ttyACM0',
            baudrate=115200,
            timeout=0.1
        )

        self.get_logger().info("Arduino Bridge connecté sur /dev/ttyACM0")

        # Thread lecture série
        self.serial_thread = threading.Thread(
            target=self.read_serial_loop,
            daemon=True
        )
        self.serial_thread.start()

    # =====================================================
    #           LECTURE USB → ROS
    # =====================================================
    def read_serial_loop(self):
        buffer = ""

        while rclpy.ok():
            try:
                if self.serial_port.in_waiting == 0:
                    continue

                chunk = self.serial_port.read(
                    self.serial_port.in_waiting
                ).decode('utf-8', errors='ignore')

                buffer += chunk

                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    self.handle_json_line(line.strip())

            except Exception as e:
                self.get_logger().error(f"Erreur série: {e}")

    def handle_json_line(self, line: str):
        try:
            data = json.loads(line)
            msg_type = data.get("type")

            if msg_type == "joystick":
                self.handle_joystick(data)

            elif msg_type == "ultrasonic":
                self.handle_ultrasonic(data)

            elif msg_type == "emergency":
                self.handle_emergency(data)

        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON invalide: {line}")

    # =====================================================
    #                  HANDLERS ENTRANTS
    # =====================================================
    def handle_joystick(self, data):
        msg = Joystick()
        msg.x = float(data["x"])
        msg.y = float(data["y"])
        self.joystick_pub.publish(msg)

    def handle_ultrasonic(self, data):
        msg = UltrasonicArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.distances = [float(d) / 100.0 for d in data["distances"]]
        self.ultrasonic_pub.publish(msg)

    def handle_emergency(self, data):
        msg = EmergencyData()
        msg.stop = bool(data["stop"])
        self.emergency_pub.publish(msg)

    # =====================================================
    #           ROS → USB (COMMANDES SERVOS)
    # =====================================================
    def servo_command_callback(self, msg: ServoCommand):
        """
        Reçoit la commande finale du MasterNode
        et l’envoie à l’Arduino en JSON.
        """
        try:
            payload = {
                "type": "servo",
                "x_angle": msg.x_angle,
                "y_angle": msg.y_angle,
                "emergency": msg.emergency_stop
            }

            json_str = json.dumps(payload) + "\n"
            self.serial_port.write(json_str.encode('utf-8'))

        except Exception as e:
            self.get_logger().error(f"Erreur envoi servo: {e}")

    # =====================================================
    #                     CLEANUP
    # =====================================================
    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
