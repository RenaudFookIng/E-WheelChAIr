#!/usr/bin/env python3
"""
Arduino Bridge Node (CSV)
==========================

Pont bidirectionnel entre Arduino (USB / CSV) et ROS 2.

Arduino → ROS :
- Joystick (angles calculés)
- Ultrasonic (3 capteurs)
- Status / READY (debug)

ROS → Arduino :
- ServoCommand (commande finale)

Frequence 20 Hz
AUCUNE logique décisionnelle ici.
"""

import rclpy
from rclpy.node import Node
import serial
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

    def __init__(self):
        super().__init__('arduino_bridge_node')

        # ---------------- PUBLISHERS ----------------
        self.joystick_pub = self.create_publisher(Joystick, '/joystick/data', 10)
        self.ultrasonic_pub = self.create_publisher(UltrasonicArray, '/ultrasonic/data', 10)
        self.emergency_pub = self.create_publisher(EmergencyData, '/emergency_data', 10)

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
        """
        Lecture des lignes envoyées par l'Arduino (CSV)
        Formats attendus :
        - Ultrasons: U,d1,d2,d3
        - Joystick:  J,angleX,angleY
        """
        buffer = ""

        while rclpy.ok():
            try:
                n = self.serial_port.in_waiting
                if n > 0:
                    chunk = self.serial_port.read(n).decode('utf-8', errors='ignore')
                    buffer += chunk

                    while '\n' in buffer:
                        line, buffer = buffer.split('\n', 1)
                        line = line.strip()
                        if line:
                            self.handle_csv_line(line)
                else:
                    time.sleep(0.002)
            except Exception as e:
                self.get_logger().error(f"Erreur série: {e}")
                time.sleep(0.1)

    # =====================================================
    #               CSV DISPATCHER
    # =====================================================
    def handle_csv_line(self, line: str):
        """
        Dispatch selon le premier caractère de la ligne CSV
        """
        if line.startswith("U,"):
            self.handle_ultrasonic_csv(line)
        elif line.startswith("J,"):
            self.handle_joystick_csv(line)
        elif line.upper() == "READY":
            self.get_logger().info("Arduino prêt")
        else:
            self.get_logger().warn(f"Ligne CSV inconnue: {line}")

    # =====================================================
    #             HANDLERS ARDUINO → ROS
    # =====================================================
    def handle_ultrasonic_csv(self, line: str):
        """
        Format attendu : U,d1,d2,d3
        """
        try:
            parts = line.split(',')
            if len(parts) != 4:
                self.get_logger().warn(f"Ultrasons mal formés: {line}")
                return

            d1, d2, d3 = map(float, parts[1:])
            msg = UltrasonicArray()
            msg.distances = [d1, d2, d3]
            self.ultrasonic_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur parsing ultrasons: {e}")

    def handle_joystick_csv(self, line: str):
        """
        Format attendu : J,angleX,angleY
        """
        try:
            parts = line.split(',')
            if len(parts) != 3:
                self.get_logger().warn(f"Joystick mal formé: {line}")
                return

            angleX, angleY = map(float, parts[1:])
            msg = Joystick()
            msg.x = angleX
            msg.y = angleY
            self.joystick_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Erreur parsing joystick: {e}")

    # =====================================================
    #               ROS → ARDUINO
    # =====================================================
    def servo_command_callback(self, msg: ServoCommand):
        """
        Reçoit la commande finale du MasterNode
        et l’envoie à l’Arduino.
        """
        try:
            # Format simple CSV pour Arduino: S,x_angle,y_angle,emergency
            emergency_int = 1 if msg.emergency_stop else 0
            csv_str = f"S,{int(msg.x_angle)},{int(msg.y_angle)},{emergency_int}\n"
            self.get_logger().info(f"Envoi commande Servo → {csv_str.strip()}")
            self.serial_port.write(csv_str.encode('utf-8'))
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
