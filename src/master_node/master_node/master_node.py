import rclpy
from rclpy.node import Node

# Permet de retrouver le dossier share/ d’un package ROS
from ament_index_python.packages import get_package_share_directory

# Messages personnalisés du projet
from custom_msgs.msg import (
    EmergencyData, 
    ObstacleDetection,
    Joystick,
    UltrasonicArray,
    ServoCommand)

# (Non utilisé actuellement, mais importé)
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray



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
        #                    SUBSCRIBERS
        # =====================================================

        # Joystick venant de arduino_data_receiver
        # msg.data = [x, y] avec x,y ∈ [-1 ; 1]
        self.create_subscription(
            Joystick,
            '/joystick/data',
            self.joystick_callback,
            10
        )

        # Données ultrasons (tableau de distances en mètres)
        self.create_subscription(
            UltrasonicArray,
            '/ultrasonic_data',
            self.ultrasonic_callback,
            10
        )

        # Arrêt d’urgence
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
        #                    PUBLISHERS
        # =====================================================

        # Commande finale envoyée au servo_controller
        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            '/servo_commands',
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

        # Paramètres servo (logique centrale)
        self.servo_neutral_x = 90
        self.servo_neutral_y = 90
        self.servo_amplitude = 30  # degrés max autour du neutre

        self.get_logger().info("Master Node started")


    # =====================================================
    #                  CALLBACKS ENTRÉES
    # =====================================================

    def joystick_callback(self, msg: Float32MultiArray):
        """
        Réception du joystick Arduino.
        C'est l'intention brute de l'utilisateur.
        """
        if len(msg.data) != 2:
            return

        self.joystick_x = float(msg.data[0])
        self.joystick_y = float(msg.data[1])

        # À CHAQUE NOUVELLE INTENTION → on redécide
        self.decide_and_publish()

    def ultrasonic_callback(self, msg: UltrasonicArray):
        """
        Réception des capteurs ultrasons.
        On garde seulement la distance minimale.
        """
        if msg.distances:
            self.obstacle_distance = min(msg.distances)

    def obstacle_callback(self, msg: ObstacleDetection):
        """
        Détection obstacle par vision.
        Écrase l’info ultrason si plus critique.
        """
        self.obstacle_distance = msg.relative_distance

    def emergency_callback(self, msg: EmergencyData):
        """
        Arrêt d’urgence = priorité ABSOLUE.
        """
        self.emergency_stop = msg.stop

        if self.emergency_stop:
            self.get_logger().warn(" EMERGENCY STOP ACTIVATED")
            self.decide_and_publish()

    # =====================================================
    #              LOGIQUE DE DÉCISION CENTRALE
    # =====================================================

    def decide_and_publish(self):
        """
        CŒUR DU SYSTÈME
        Cette fonction applique TOUTES les règles :
        1. arrêt d’urgence
        2. évitement d’obstacle
        3. fonctionnement normal
        """

        # ===============================
        # CAS 1 : ARRÊT D’URGENCE
        # ===============================
        if self.emergency_stop:
            x_angle = self.servo_neutral_x
            y_angle = self.servo_neutral_y

            self.publish_servo_command(
                x_angle, y_angle,
                source="emergency"
            )
            return

        # ===============================
        # CAS 2 : OBSTACLE TROP PROCHE
        # ===============================
        if self.obstacle_distance < 0.30:
            # On bloque l’avance, mais on autorise la rotation
            x_angle = self.map_normalized_to_servo(
                self.joystick_x,
                self.servo_neutral_x
            )
            y_angle = self.servo_neutral_y

            self.publish_servo_command(
                x_angle, y_angle,
                source="obstacle_avoidance"
            )
            return

        # ===============================
        # CAS 3 : FONCTIONNEMENT NORMAL
        # ===============================
        x_angle = self.map_normalized_to_servo(
            self.joystick_x,
            self.servo_neutral_x
        )

        y_angle = self.map_normalized_to_servo(
            self.joystick_y,
            self.servo_neutral_y
        )

        self.publish_servo_command(
            x_angle, y_angle,
            source="joystick"
        )

    # =====================================================
    #            OUTILS DE CONVERSION
    # =====================================================

    def map_normalized_to_servo(self, value, neutral):
        """
        Convertit une valeur normalisée [-1 ; 1]
        en angle servo autour d’un neutre.

        Exemple :
        value = -1  → neutral - amplitude
        value =  0  → neutral
        value = +1  → neutral + amplitude
        """
        angle = neutral + int(value * self.servo_amplitude)
        return max(0, min(180, angle))

    # =====================================================
    #           PUBLICATION COMMANDE SERVO
    # =====================================================

    def publish_servo_command(self, x_angle, y_angle, source):
        """
        Publie la COMMANDE FINALE vers le servo_controller.
        """
        msg = ServoCommand()

        msg.x_normalized = self.joystick_x
        msg.y_normalized = self.joystick_y

        msg.x_angle = x_angle
        msg.y_angle = y_angle

        msg.emergency_stop = self.emergency_stop
        msg.stamp = self.get_clock().now().to_msg()

        self.servo_command_pub.publish(msg)

        self.get_logger().debug(
            f"[{source}] Servo command → X:{x_angle} Y:{y_angle}"
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
