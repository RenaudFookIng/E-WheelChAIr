import rclpy
from rclpy.node import Node

# Permet de retrouver le dossier share/ d’un package ROS
from ament_index_python.packages import get_package_share_directory

# Messages personnalisés du projet
from custom_msgs.msg import (
    EmergencyData, 
    ObstacleDetection,
    VisionObstacle,
    Joystick,
    UltrasonicArray,
    ServoCommand
)


# (Non utilisé actuellement, mais importé)
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray



class MasterNode(Node):
    """
    MASTER NODE
    ===========
    Nœud central du fauteuil roulant.
    Rôles :
    LOGIQUE DES PRIORITÉS :
    1. Emergency Stop → priorité absolue → bloque tous les mouvements
    2. Joystick actif → source principale du mouvement
    3. Front obstacle → modificateur pour l’avance
    4. Rear obstacle (ultrasons) → modificateur pour la marche arrière
    """

    def __init__(self):
        super().__init__('master_node')

        # =====================================================
        #                  PARAMÈTRES CONFIGURABLES
        # =====================================================
        self.declare_parameter("front_obstacle_caution_distance", 0.30)
        self.declare_parameter("front_obstacle_stop_distance", 0.15)
        self.declare_parameter("rear_obstacle_stop_distance", 0.20)
        self.declare_parameter("avoidance_gain", 0.5)

        self.front_obstacle_caution_distance = self.get_parameter(
            "front_obstacle_caution_distance").value
        self.front_obstacle_stop_distance = self.get_parameter(
            "front_obstacle_stop_distance").value
        self.rear_obstacle_stop_distance = self.get_parameter(
            "rear_obstacle_stop_distance").value
        self.avoidance_gain = self.get_parameter(
            "avoidance_gain").value

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
            '/ultrasonic/data',
            self.ultrasonic_callback,
            10
        )

        # Vision Semantique
        self.create_subscription(
            VisionObstacle,
            '/vision/obstacle',
            self.vision_obstacle_callback,
            10
        )

        # Détection d’obstacles par Camera profondeur
        self.create_subscription(
            ObstacleDetection,
            '/obstacle_detection',
            self.front_obstacle_callback,
            10
        )

        self.create_subscription(
            EmergencyData,
            '/emergency_data',
            self.emergency_callback,
            10)

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

        self.front_obstacle_distance = 999.0
        self.rear_obstacle_distance = 999.0
        self.vision_obstacle = None

        # Paramètres servo (logique centrale)
        self.servo_neutral_x = 90
        self.servo_neutral_y = 90
        self.servo_amplitude = 30  # degrés max autour du neutre

        self.get_logger().info("Master Node started")


    # =====================================================
    #                  CALLBACKS ENTRÉES
    # =====================================================
    def joystick_callback(self, msg: Joystick):
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
        """Ultrasons arrière → marche arrière"""
        if msg.distances:
            self.rear_obstacle_distance = min(msg.distances)

    def front_obstacle_callback(self, msg: ObstacleDetection):
        """Depth camera frontale → avance"""
        self.front_obstacle_distance = msg.relative_distance

    def vision_obstacle_callback(self, msg: VisionObstacle):
        if msg.detection_confidence < 0.5:
            self.vision_obstacle = None
        else:
            self.vision_obstacle = msg
            self.front_obstacle_distance = msg.forward_offset

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
        Priorités :
        1️⃣ Arrêt d'urgence (max)
        2️⃣ Obstacles avant (vision)
        3️⃣ Obstacles arrière (ultrasons)
        4️⃣ Commande joystick
        """

        # ===============================
        # PRIORITÉ 1 : ARRÊT D’URGENCE
        # ===============================
        if self.emergency_stop:
            self.publish_servo_command(
                self.servo_neutral_x,
                self.servo_neutral_y,
                source="emergency"
            )
            return

        # ===============================
        # PRIORITÉ 2 : OBSTACLE AVANT
        # ===============================
        corrected_x = self.joystick_x
        y_command = self.joystick_y

        if self.joystick_y > 0 and self.front_obstacle_distance < self.front_obstacle_caution_distance:
            # Correction de direction
            if self.vision_obstacle:
                # obstacle à droite → tourner à gauche
                corrected_x -= self.avoidance_gain * self.vision_obstacle.lateral_offset

            # Blocage complet si obstacle trop proche
            if self.front_obstacle_distance < self.front_obstacle_stop_distance:
                y_command = 0.0

        # ===============================
        # PRIORITÉ 3 : OBSTACLE ARRIÈRE
        # ===============================
        if self.joystick_y < 0 and self.rear_obstacle_distance < self.rear_obstacle_stop_distance:
            y_command = 0.0

        # ===============================
        # PRIORITÉ 4 : FONCTIONNEMENT NORMAL
        # ===============================
        x_angle = self.map_normalized_to_servo(corrected_x, self.servo_neutral_x)
        y_angle = self.map_normalized_to_servo(y_command, self.servo_neutral_y)

        self.publish_servo_command(x_angle, y_angle, source="joystick_or_avoidance")


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
