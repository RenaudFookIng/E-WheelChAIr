#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from custom_msgs.msg import WyesIntent  # Import depuis custom_msgs
import sys, select, tty, termios

class WyesTeleopNode(Node):
    def __init__(self):
        super().__init__('wyes_teleop')
        self.publisher = self.create_publisher(WyesIntent, '/wyes_intent', 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.current_key_sequence = []

    def get_key(self):
        """Lit une touche du clavier sans attendre Entrée."""
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """Boucle principale pour lire les entrées clavier et publier les intentions."""
        print("Utilise les flèches pour envoyer des intentions. 'q' pour quitter.")
        while rclpy.ok():
            key = self.get_key()
            msg = WyesIntent()
            if key == 'q':
                break
            elif key == '\x1b':  # Début d'une séquence de flèche
                self.current_key_sequence = [key]
            elif len(self.current_key_sequence) > 0:
                self.current_key_sequence.append(key)
                if len(self.current_key_sequence) == 3:  # Séquence complète (ex: \x1b[A)
                    if self.current_key_sequence[1] == '[':
                        if self.current_key_sequence[2] == 'A':  # Flèche haut
                            msg.direction = "forward"
                        elif self.current_key_sequence[2] == 'B':  # Flèche bas
                            msg.direction = "backward"
                        elif self.current_key_sequence[2] == 'C':  # Flèche droite
                            msg.direction = "right"
                        elif self.current_key_sequence[2] == 'D':  # Flèche gauche
                            msg.direction = "left"
                    self.current_key_sequence = []
            else:
                msg.direction = "stop"
            self.publisher.publish(msg)
            rclpy.spin_once(self, timeout_sec=0.1)

def main(args=None):
    rclpy.init(args=args)
    node = WyesTeleopNode()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
