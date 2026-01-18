#!/usr/bin/env python3

"""
Arduino Data Receiver Node
==========================

Ce nœud est un TRADUCTEUR entre :
- le monde matériel (Arduino, USB, JSON)
- le monde ROS (messages typés et propres)

Il NE CONTIENT AUCUNE LOGIQUE DE DÉCISION.
"""

import rclpy
from rclpy.node import Node

import serial
import json
import threading

# ===============================
#        MESSAGES CUSTOM
# ===============================
from custom_msgs.msg import UltrasonicArray, Joystick, ServoCommand, EmergencyData


class ArduinoDataReceiverNode(Node):
    def __init__(self):
        super().__init__('arduino_data_receiver_node')

        # Publishers
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

        # =====================================================
        #                 CONNEXION SÉRIE
        # =====================================================

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
    
    # =====================================================
    #              BOUCLE LECTURE SÉRIE
    # =====================================================
    def read_serial_loop(self):
        """
        Lit en continu le port série Arduino.
        Chaque ligne doit être un JSON valide.
        """
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

    # =====================================================
    #                 JOYSTICK
    # =====================================================
    def handle_joystick(self, data):
        try:
            msg = Joystick()
            msg.x = float(data["x"])
            msg.y = float(data["y"])
            self.joystick_pub.publish(msg)
        except KeyError:
            self.get_logger().warn("Joystick: données manquantes")

    # =====================================================
    #                 ULTRASOUNDS
    # =====================================================
    def handle_ultrasonic(self, data):
        """
        Convertit les capteurs ultrasons en UltrasonicArray.msg
        """
        try:
            msg = UltrasonicArray()
            msg.header.stamp = self.get_clock().now().to_msg()

            # Conversion cm → mètres
            msg.distances = [
                float(d) / 100.0 for d in data["sensors"]
            ]

            self.ultrasonic_pub.publish(msg)

        except KeyError:
            self.get_logger().warn("Ultrasonic: données manquantes")

    # =====================================================
    #                       CLEANUP
    # =====================================================

    def destroy_node(self):
        if self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoDataReceiverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
