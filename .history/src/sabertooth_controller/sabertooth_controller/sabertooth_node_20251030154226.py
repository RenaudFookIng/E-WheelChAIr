#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sabertooth import Sabertooth

class SabertoothController(Node):
    def __init__(self):
        super().__init__('sabertooth_controller')
        self.subscription = self.create_subscription(
            String, 'arduino_data', self.listener_callback, 10)
        self.sabertooth = Sabertooth('/dev/ttyUSB0', baudrate=9600)
        self.get_logger().info("Sabertooth Controller démarré.")

    def listener_callback(self, msg):
        try:
            valeurs = msg.data.split(',')
            valeurX = int(valeurs[0])
            valeurY = int(valeurs[1])
            distance = int(valeurs[3])

            vitesse_gauche = self.map_valeur(valeurY, 0, 1023, -100, 100)
            vitesse_droite = self.map_valeur(valeurX, 0, 1023, -100, 100)

            self.sabertooth.drive(1, vitesse_gauche)
            self.sabertooth.drive(2, vitesse_droite)

            if distance < 30:
                self.sabertooth.stop()
                self.get_logger().warn("Obstacle détecté ! Arrêt des moteurs.")

        except Exception as e:
            self.get_logger().error(f"Erreur : {e}")

    def map_valeur(self, valeur, in_min, in_max, out_min, out_max):
        return (valeur - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = SabertoothController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
