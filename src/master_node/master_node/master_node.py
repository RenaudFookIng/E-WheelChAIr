import rclpy
from rclpy.node import Node

# Communication série avec les Arduino (servo + joystick)
import serial

# Lecture des fichiers YAML de configuration
import yaml
import os

# Permet de retrouver le dossier share/ d’un package ROS
from ament_index_python.packages import get_package_share_directory

# Messages personnalisés du projet
from custom_msgs.msg import UltrasonicArray, Joystick, EmergencyData, ObstacleDetection

# (Non utilisé actuellement, mais importé)
from geometry_msgs.msg import Twist


class MasterNode(Node):
    """
    MASTER NODE
    ===========
    Nœud central du fauteuil roulant.

    Rôles :
    - Lire les intentions utilisateur (joystick Arduino)
    - Recevoir les capteurs (ultrasons, vision, arrêt d'urgence)
    - Décider si le mouvement est autorisé
    - Envoyer les commandes finales au contrôleur de servos
    """

    def __init__(self):
        super().__init__('master_node')

        # =====================================================
        #                       PUBLISHERS
        # =====================================================

        # Publication de la commande joystick VALIDÉE
        # (après sécurité et évitement)
        self.joystick_pub = self.create_publisher(
            Joystick,
            '/joystick_input',
            10
        )

        # =====================================================
        #                      SUBSCRIBERS
        # =====================================================

        # Données ultrasons (distance en mètres)
        self.create_subscription(
            UltrasonicArray,
            '/ultrasonic_data',
            self.ultrasonic_callback,
            10
        )

        # Données arrêt d'urgence
        self.create_subscription(
            EmergencyData,
            '/emergency_data',
            self.emergency_callback,
            10
        )

        # Détection d’obstacles par vision
        self.create_subscription(
            ObstacleDetection,
            '/obstacle_detection',
            self.obstacle_callback,
            10
        )

        # =====================================================
        #                  ÉTAT INTERNE DU SYSTÈME
        # =====================================================

        # Indique si un arrêt d'urgence est actif
        self.emergency_stop = False

        # Intention utilisateur normalisée [-1.0 ; 1.0]
        # x : rotation (gauche/droite)
        # y : translation (avant/arrière)
        self.joystick_x = 0.0
        self.joystick_y = 0.0

        # Distance minimale détectée (ultrasons ou vision)
        self.obstacle_distance = 999.0

        # Autorisation d'envoyer des commandes aux servos
        self.servo_enabled = True

        # =====================================================
        #           INITIALISATION DES COMMUNICATIONS
        # =====================================================

        # Connexion au contrôleur de servos (Arduino)
        self.setup_servo_controller()

        # Connexion au joystick Arduino
        self.setup_arduino_joystick()

        self.get_logger().info('Master Node started and ready.')

    # =========================================================
    #           CONNEXION AU CONTROLEUR DE SERVOS
    # =========================================================

    def setup_servo_controller(self):
        """
        Initialise la communication série avec l’Arduino
        qui pilote les servos du fauteuil.
        """
        try:
            # Récupération du fichier de configuration
            config_path = os.path.join(
                get_package_share_directory('master_node'),
                'config',
                'servo_config.yaml'
            )

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Ouverture du port série vers le contrôleur de servos
            self.servo_serial = serial.Serial(
                port=config['servo_controller']['port'],
                baudrate=config['servo_controller']['baudrate'],
                timeout=1
            )

            self.get_logger().info(
                f"Servo controller connected on {config['servo_controller']['port']}"
            )

        except Exception as e:
            # En cas d'erreur : désactivation complète des servos
            self.get_logger().error(
                f"Failed to connect to servo controller: {e}"
            )
            self.servo_enabled = False

    # =========================================================
    #              CONNEXION AU JOYSTICK ARDUINO
    # =========================================================

    def setup_arduino_joystick(self):
        """
        Initialise la communication série avec l’Arduino joystick.
        Le joystick n’est PAS un nœud ROS.
        """
        try:
            config_path = os.path.join(
                get_package_share_directory('master_node'),
                'config',
                'servo_config.yaml'
            )

            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Hypothèse matérielle :
            # - Servo controller : /dev/ttyACM1
            # - Joystick Arduino : /dev/ttyACM0
            joystick_port = config['servo_controller']['port'].replace(
                'ACM1', 'ACM0'
            )

            self.joystick_serial = serial.Serial(
                port=joystick_port,
                baudrate=115200,
                timeout=1
            )

            self.get_logger().info(
                f"Arduino joystick connected on {joystick_port}"
            )

            # Lecture du joystick toutes les 50 ms (20 Hz)
            self.joystick_timer = self.create_timer(
                0.05,
                self.read_arduino_joystick
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to connect to Arduino joystick: {e}"
            )

    # =========================================================
    #                    CALLBACKS CAPTEURS
    # =========================================================

    def ultrasonic_callback(self, msg: UltrasonicArray):
        """
        Réception des distances ultrasons.
        On conserve uniquement la distance la plus proche.
        """
        if msg.distances:
            self.obstacle_distance = min(msg.distances)

    def emergency_callback(self, msg: EmergencyData):
        """
        Arrêt d’urgence : priorité absolue.
        """
        self.emergency_stop = msg.stop

        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated!')
            # Forcer immédiatement l’arrêt
            self.publish_joystick_command()

    def obstacle_callback(self, msg: ObstacleDetection):
        """
        Obstacle détecté par vision.
        Écrase la distance ultrason actuelle.
        """
        self.obstacle_distance = msg.relative_distance

    # =========================================================
    #                LOGIQUE DÉCISIONNELLE
    # =========================================================

    def publish_joystick_command(self):
        """
        Fonction centrale de décision.

        Elle applique :
        - arrêt d'urgence
        - évitement d'obstacles
        - ou fonctionnement normal
        """

        # ----- CAS 1 : ARRÊT D’URGENCE -----
        if self.emergency_stop:
            # Position neutre des servos = fauteuil immobile
            self.get_logger().warn(
                'Emergency stop activated! Servos moved to neutral.'
            )
            self.send_servo_command(90, 90)

        # ----- CAS 2 : OBSTACLE TROP PROCHE -----
        elif self.obstacle_distance < 0.3:
            # Autorise la rotation mais bloque l’avance
            self.get_logger().info('Obstacle detected - stopping')

            x_angle = self.normalize_to_servo_angle(self.joystick_x)
            y_angle = 90  # neutre avant/arrière

            self.send_servo_command(x_angle, y_angle)

        # ----- CAS 3 : FONCTIONNEMENT NORMAL -----
        else:
            x_angle = self.normalize_to_servo_angle(self.joystick_x)
            y_angle = self.normalize_to_servo_angle(self.joystick_y)

            self.send_servo_command(x_angle, y_angle)

    # =========================================================
    #        CONVERSION JOYSTICK → ANGLE SERVO
    # =========================================================

    def normalize_to_servo_angle(self, value):
        """
        Convertit une valeur normalisée [-1 ; 1]
        vers un angle servo [0 ; 180]
        """
        return int((value + 1.0) * 90.0)

    # =========================================================
    #               ENVOI COMMANDE SERVO
    # =========================================================

    def send_servo_command(self, x_angle, y_angle):
        """
        Envoie la commande finale au contrôleur de servos
        et publie la commande validée sur ROS.
        """

        if not self.servo_enabled or not hasattr(self, 'servo_serial'):
            return

        try:
            # Sécurisation des angles
            x_angle = max(0, min(180, x_angle))
            y_angle = max(0, min(180, y_angle))

            # Protocole série simple
            command = f"SERVO,{x_angle},{y_angle}\n"
            self.servo_serial.write(command.encode())

            # Publication ROS de la commande validée
            joystick_msg = Joystick()
            joystick_msg.x = self.joystick_x
            joystick_msg.y = self.joystick_y
            self.joystick_pub.publish(joystick_msg)

            self.get_logger().debug(
                f"Sent servo command: X={x_angle}°, Y={y_angle}°"
            )

        except Exception as e:
            self.get_logger().error(
                f"Failed to send servo command: {e}"
            )
            self.servo_enabled = False

    # =========================================================
    #            LECTURE JOYSTICK ARDUINO
    # =========================================================

    def read_arduino_joystick(self):
        """
        Lecture du joystick via la liaison série.
        Format attendu :
        x,y
        """

        if not hasattr(self, 'joystick_serial') or not self.joystick_serial.is_open:
            return

        try:
            line = self.joystick_serial.readline().decode(
                'utf-8'
            ).strip()

            if line:
                parts = line.split(',')
                if len(parts) == 2:
                    self.joystick_x = float(parts[0].strip())
                    self.joystick_y = float(parts[1].strip())

                    # Déclenchement immédiat de la décision
                    self.publish_joystick_command()

        except Exception as e:
            self.get_logger().error(
                f"Error reading Arduino joystick: {e}"
            )

    # =========================================================
    #                        MAIN
    # =========================================================


def main(args=None):
    rclpy.init(args=args)

    node = MasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Fermeture propre des ports série
        if hasattr(node, 'servo_serial'):
            node.servo_serial.close()

        if hasattr(node, 'joystick_serial'):
            node.joystick_serial.close()

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
