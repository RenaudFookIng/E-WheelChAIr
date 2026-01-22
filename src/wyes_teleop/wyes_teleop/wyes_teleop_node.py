#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import WyesIntent
import sys
import select
import tty
import termios
import time


class WyesTeleopNode(Node):
    def __init__(self):
        super().__init__('wyes_teleop')

        self.publisher = self.create_publisher(WyesIntent, '/wyes_intent', 10)

        # Détection TTY (important pour launch)
        self.has_tty = sys.stdin.isatty()

        if self.has_tty:
            self.settings = termios.tcgetattr(sys.stdin)
            self.get_logger().info("TTY détecté — contrôle clavier actif")
        else:
            self.get_logger().warn("Pas de TTY — téléop clavier désactivée (mode launch)")

        self.current_key_sequence = []
        self.last_command_time = time.time()
        self.command_timeout = 0.25  # STOP si aucune touche depuis 250 ms
        self.last_direction = "stop"

        self.get_logger().info("Utilise flèches / WASD / Entrée pour piloter — 'q' pour quitter")


    # ============================
    # Lecture clavier sécurisée
    # ============================
    def get_key(self):
        if not self.has_tty:
            return ''

        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.05)

        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key


    # ============================
    # Boucle principale
    # ============================
    def run(self):
        rate = self.create_rate(20)  # 20 Hz

        while rclpy.ok():
            key = self.get_key()
            msg = WyesIntent()
            direction = None

            # Quitter
            if key == 'q':
                self.get_logger().warn("Sortie Wyes Teleop demandée")
                break

            # ============================
            # TOUCHE ENTRÉE = AVANT
            # ============================
            elif key == '\r' or key == '\n':
                direction = "forward"
                self.get_logger().info("[KEY] ENTER → FORWARD")

            # ============================
            # GESTION DES FLÈCHES
            # ============================
            elif key == '\x1b':  
                self.current_key_sequence = [key]

            elif len(self.current_key_sequence) > 0:
                self.current_key_sequence.append(key)

                if len(self.current_key_sequence) == 3:
                    if self.current_key_sequence[1] == '[':
                        arrow = self.current_key_sequence[2]

                        if arrow == 'A':
                            direction = "forward"
                            self.get_logger().info("[KEY] FLÈCHE HAUT → FORWARD")

                        elif arrow == 'B':
                            direction = "backward"
                            self.get_logger().info("[KEY] FLÈCHE BAS → BACKWARD")

                        elif arrow == 'C':
                            direction = "right"
                            self.get_logger().info("[KEY] FLÈCHE DROITE → RIGHT")

                        elif arrow == 'D':
                            direction = "left"
                            self.get_logger().info("[KEY] FLÈCHE GAUCHE → LEFT")

                    self.current_key_sequence = []

            # ============================
            # TOUCHES WASD OPTIONNELLES
            # ============================
            elif key.lower() == 'w':
                direction = "forward"
            elif key.lower() == 's':
                direction = "backward"
            elif key.lower() == 'a':
                direction = "left"
            elif key.lower() == 'd':
                direction = "right"

            # ============================
            # APPLICATION DE LA COMMANDE
            # ============================
            now = time.time()

            if direction:
                msg.direction = direction
                self.last_direction = direction
                self.last_command_time = now
                self.publisher.publish(msg)

                self.get_logger().debug(f"[PUBLISH] direction = {direction}")

            # ============================
            # WATCHDOG STOP AUTO
            # ============================
            elif now - self.last_command_time > self.command_timeout:
                if self.last_direction != "stop":
                    msg.direction = "stop"
                    self.publisher.publish(msg)
                    self.last_direction = "stop"

                    self.get_logger().warn("[WATCHDOG] STOP — aucune touche reçue")

            rclpy.spin_once(self, timeout_sec=0.01)
            rate.sleep()


# ============================
# MAIN
# ============================
def main(args=None):
    rclpy.init(args=args)

    node = WyesTeleopNode()
    node.run()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
