#!/usr/bin/env python3
"""
Wide Processing Node
====================
Ce nœud ROS 2 :
- reçoit des données de détection depuis un ordinateur externe via USB (port série)
- parse les données JSON
- publie les informations sous forme de messages ROS ObstacleDetection
AUCUN traitement d'image n'est effectué ici.
Le PC externe fait 100 % du calcul vision.
"""

import rclpy
from rclpy.node import Node
import serial
import json

from custom_msgs.msg import ObstacleDetection


class WideProcessingNode(Node):
    def __init__(self):
        super().__init__('wide_processing_node')

        # -------- PARAMÈTRES USB --------
        self.declare_parameter('usb_port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)

        port = self.get_parameter('usb_port').get_parameter_value().string_value
        baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value

        # -------- PUBLISHER ROS --------
        self.publisher_ = self.create_publisher(
            ObstacleDetection,
            '/wide_detected_objects',
            10
        )

        # -------- CONNEXION USB --------
        try:
            self.serial = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f"Wide camera USB connected on {port}")
        except Exception as e:
            self.get_logger().error(f"Impossible d'ouvrir le port USB : {e}")
            self.serial = None

        # -------- TIMER DE LECTURE --------
        self.timer = self.create_timer(0.02, self.read_usb_data)

    def read_usb_data(self):
        """
        Lecture non bloquante du port USB.
        Chaque ligne doit être un JSON valide.
        """
        if self.serial is None or not self.serial.is_open:
            return

        try:
            line = self.serial.readline().decode('utf-8').strip()
            if not line:
                return

            data = json.loads(line)

            msg = ObstacleDetection()
            msg.is_human = (data.get('type') == 'human')
            msg.relative_x = float(data.get('x', 0.0))
            msg.relative_y = float(data.get('y', 0.0))
            msg.relative_distance = float(data.get('distance', 999.0))

            self.publisher_.publish(msg)

            self.get_logger().debug(
                f"WIDE → human={msg.is_human} "
                f"x={msg.relative_x:.2f} "
                f"y={msg.relative_y:.2f} "
                f"d={msg.relative_distance:.2f}"
            )

        except json.JSONDecodeError:
            self.get_logger().warn(f"Donnée invalide reçue : {line}")
        except Exception as e:
            self.get_logger().error(f"Erreur USB wide camera : {e}")


def main(args=None):
    rclpy.init(args=args)
    node = WideProcessingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.serial and node.serial.is_open:
            node.serial.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
