#!/usr/bin/env python3
"""
Arduino Bridge Node (JSON)
==========================

Pont bidirectionnel entre Arduino (USB / JSON) et ROS 2.

Arduino → ROS :
- joystick
- ultrasonic
- emergency
- status (debug)

ROS → Arduino :
- ServoCommand (commande finale uniquement)

AUCUNE logique décisionnelle ici.
"""

import rclpy
from rclpy.node import Node

import serial
import json
import threading
import time

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

    # =====================================================
    #                      INIT
    # =====================================================
    def __init__(self):
        super().__init__('arduino_bridge_node')

        # ---------------- PUBLISHERS ----------------
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

        # ---------------- SUBSCRIBERS ----------------
        self.create_subscription(
            ServoCommand,
            '/servo_commands',
            self.servo_command_callback,
            10
        )

        # ---------------- SERIAL ----------------
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
    #            SERIAL → ROS (LECTURE)
    # =====================================================
    def read_serial_loop(self):
        buffer = ""

        while rclpy.ok():
            try:
                # Lire uniquement si des octets sont disponibles
                n = self.serial_port.in_waiting
                if n > 0:
                    chunk = self.serial_port.read(n).decode('utf-8', errors='ignore')
                    buffer += chunk

                    # Traiter toutes les lignes complètes
                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        #self.get_logger().info(f"Ligne reçue: {line}")
                        if line:  # éviter les lignes vides
                            self.handle_json_line(line)
                else:
                    time.sleep(0.002)  # Attente active pour ne pas saturer le CPU

            except Exception as e:
                self.get_logger().error(f"Erreur série: {e}")
                time.sleep(0.1)  # Petite pause pour éviter flood d'erreurs


    # =====================================================
    #                JSON DISPATCHER
    # =====================================================
    def handle_json_line(self, line: str):
        if not line:
            return

        try:
            data = json.loads(line)
            msg_type = data.get("type", "")

            if msg_type == "joystick":
                self.handle_joystick(data)

            elif msg_type == "ultrasonic":
                self.handle_ultrasonic(data)

            elif msg_type == "emergency":
                self.handle_emergency(data)

            elif msg_type == "status":
                self.handle_status(data)

            elif msg_type == "ready":
                self.get_logger().info("Arduino prêt")

            else:
                self.get_logger().warn(f"Type JSON inconnu: {msg_type}")

        except json.JSONDecodeError:
            self.get_logger().warn(f"JSON invalide: {line}")

    # =====================================================
    #               HANDLERS ARDUINO → ROS
    # =====================================================
    def handle_joystick(self, data):
        msg = Joystick()
        msg.x = float(data.get("x", 0.0))
        msg.y = float(data.get("y", 0.0))
        self.joystick_pub.publish(msg)

    def handle_ultrasonic(self, data):
        msg = UltrasonicArray()
        #msg.header.stamp = self.get_clock().now().to_msg()
        msg.distances = [float(d) for d in data.get("distances", [])]
        self.ultrasonic_pub.publish(msg)

    def handle_emergency(self, data):
        msg = EmergencyData()
        msg.stop = (data.get("status", "") == "stopped")
        self.emergency_pub.publish(msg)

        if msg.stop:
            self.get_logger().error("🚨 EMERGENCY STOP REÇU DE L'ARDUINO")

    def handle_status(self, data):
        self.get_logger().debug(
            f"Servo status | X={data.get('currentX')} "
            f"Y={data.get('currentY')} "
            f"Amp={data.get('amplitude')}"
        )

    # =====================================================
    #               ROS → ARDUINO
    # =====================================================
    def servo_command_callback(self, msg: ServoCommand):
        """
        Reçoit la commande finale du MasterNode
        et l’envoie à l’Arduino en JSON.
        """
        try:
            payload = {
                "type": "servo",
                "x_angle": int(msg.x_angle),
                "y_angle": int(msg.y_angle),
                "emergency": bool(msg.emergency_stop)
            }

            json_str = json.dumps(payload) + "\n"
            self.get_logger().info(f"Envoi JSON Servo → {json_str.strip()}")
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


# =====================================================
#                      MAIN
# =====================================================
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
