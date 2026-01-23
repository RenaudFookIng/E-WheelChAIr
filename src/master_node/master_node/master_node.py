import rclpy
import yaml
import os
from rclpy.node import Node

# Permet de retrouver le dossier share/ d’un package ROS
from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('master_node')

master_config_file = os.path.join(
    pkg_share, 'config', 'master_config.yaml'
)
servo_config_file = os.path.join(
    pkg_share, 'config', 'servo_config.yaml'
)

# Messages personnalisés du projet
from custom_msgs.msg import (
    EmergencyData, 
    ObstacleDetection,
    VisionObstacle,
    Joystick,
    UltrasonicArray,
    ServoCommand,
    WyesIntent
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
        # ===========================
        # Chargement des configs
        # ===========================
        with open(master_config_file, 'r') as f:
            master_cfg = yaml.safe_load(f)

        with open(servo_config_file, 'r') as f:
            servo_cfg = yaml.safe_load(f)

        # ===========================
        # Paramètres de sécurité
        # ===========================
        safety = master_cfg.get('safety', {})
        self.emergency_stop_timeout = safety.get('emergency_stop_timeout', 0.5)
        self.front_obstacle_caution_distance = safety.get('front_obstacle_caution_distance', 0.5)
        self.front_obstacle_stop_distance = safety.get('front_obstacle_stop_distance', 0.2)
        self.rear_obstacle_stop_distance = safety.get('rear_obstacle_stop_distance', 0.2)
        self.max_speed = safety.get('max_speed', 1.0)

        # ===========================
        # Paramètres servos
        # ===========================
        self.servo_neutral_x = servo_cfg['servo_x']['neutral']  # 85
        self.servo_neutral_y = servo_cfg['servo_y']['neutral']  # 90
        self.servo_min_x = servo_cfg['servo_x']['min']          # 70
        self.servo_max_x = servo_cfg['servo_x']['max']          # 100
        self.servo_min_y = servo_cfg['servo_y']['min']          # 75
        self.servo_max_y = servo_cfg['servo_y']['max']          # 105
        self.servo_amplitude = servo_cfg.get('amplitude', 15)  # 15


        # ===========================
        # Initialisation des états
        # ===========================
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.front_obstacle_distance = 1.0
        self.rear_obstacle_distance = 1.0
        self.vision_obstacle = None
        self.emergency_stop = False

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

        # Subscription WyesIntent
        self.wyes_direction = "stop"
        self.create_subscription(
            WyesIntent,
            '/wyes_intent',
            self.wyes_intent_callback,
            10
        )

        # ===========================
        # Log info
        # ===========================
        self.get_logger().info("Master Node initialized with YAML configs")

        # =====================================================
        #                    PUBLISHERS
        # =====================================================

        # Commande finale envoyée au servo_controller
        self.servo_command_pub = self.create_publisher(
            ServoCommand,
            '/servo_commands',
            10
        )

        self.get_logger().info("Master Node started")


    # =====================================================
    #                  CALLBACKS ENTRÉES
    # =====================================================
    def joystick_callback(self, msg: Joystick):
        """
        Réception du joystick Arduino.
        C'est l'intention brute de l'utilisateur.
        """
        # On vérifie que x et y existent
        if msg.x is None or msg.y is None:
            return

        self.joystick_x = float(msg.x)
        self.joystick_y = float(msg.y)

        # --- Debug log ---
        #self.get_logger().info(
        #    f"Joystick reçu: x={self.joystick_x:.3f}, y={self.joystick_y:.3f}"
        #)

        # À CHAQUE NOUVELLE INTENTION → on redécide
        self.decide_and_publish()

    def ultrasonic_callback(self, msg: UltrasonicArray):
        """Ultrasons arrière → marche arrière"""
        # Vérifie que la liste n'est pas vide
        if len(msg.distances) > 0:
            # Exemple : prendre la distance minimale pour obstacle arrière
            self.rear_obstacle_distance = min(msg.distances)
            # --- Debug log ---
            #self.get_logger().info(
            #    f"Rear obstacle distance: {self.rear_obstacle_distance:.2f} m"
            #)

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

    def wyes_intent_callback(self, msg: WyesIntent):
        """
        Récupère la direction depuis le téléop clavier Wyes.
        Si joystick inactif, cette commande sera appliquée.
        """
        if msg.direction:
            self.wyes_direction = msg.direction
            self.get_logger().debug(f"[WYES] Direction reçue: {self.wyes_direction}")
            self.decide_and_publish()  # republier dès que nouveau message


    # =====================================================
    #              LOGIQUE DE DÉCISION CENTRALE
    # =====================================================

    def decide_and_publish(self):
        """
        CŒUR DU SYSTÈME
        Applique toutes les règles et publie la commande finale aux servos.
        Priorités :
        1️⃣ Arrêt d'urgence
        2️⃣ Obstacles avant (vision)
        3️⃣ Obstacles arrière (ultrasons)
        4️⃣ Commande joystick normale
        5️⃣ WyesIntent → si joystick inactif
        """

        # ===============================
        # PRIORITÉ 1 : ARRÊT D’URGENCE
        # ===============================
        if self.emergency_stop:
            x_angle = self.servo_neutral_x
            y_angle = self.servo_neutral_y
            self.get_logger().info("[EMERGENCY] Stop immédiat")
            self.publish_servo_command(x_angle, y_angle, source="emergency")
            return

        # ===============================
        # PRIORITÉ 2 : OBSTACLE AVANT
        # ===============================
        corrected_x = self.joystick_x
        y_command = self.joystick_y

        # si joystick inactif, on prendra WyesIntent plus bas
        joystick_active = abs(self.joystick_x) > 0.01 or abs(self.joystick_y) > 0.01

        if joystick_active:
            source = "joystick"
        else:
            # utilisation WyesIntent si joystick inactif
            mapping = {
                "forward": 1.0,
                "backward": -1.0,
                "left": -0.5,
                "right": 0.5,
                "stop": 0.0
            }
            corrected_x = mapping.get(self.wyes_direction, 0.0)
            y_command = mapping.get(self.wyes_direction, 0.0)
            source = "wyes"

        if y_command > 0 and self.front_obstacle_distance < self.front_obstacle_caution_distance:
            if self.vision_obstacle:
                corrected_x -= self.avoidance_gain * self.vision_obstacle.lateral_offset
                self.get_logger().debug(
                    f"[AVANT] Correction direction: {corrected_x:.3f} (obstacle lat: {self.vision_obstacle.lateral_offset:.3f})"
                )
            if self.front_obstacle_distance < self.front_obstacle_stop_distance:
                y_command = 0.0
                self.get_logger().info(f"[AVANT] Blocage complet, obstacle trop proche: {self.front_obstacle_distance:.2f} m")

        # ===============================
        # PRIORITÉ 3 : OBSTACLE ARRIÈRE
        # ===============================
        if self.joystick_y < 0 and self.rear_obstacle_distance < self.rear_obstacle_stop_distance:
            y_command = 0.0
            self.get_logger().info(f"[ARRIÈRE] Blocage marche arrière, obstacle à {self.rear_obstacle_distance:.2f} m")

        # ===============================
        # PRIORITÉ 4 : COMMANDE NORMALE
        # ===============================
        # Conversion normalisée → angle servo avec respect des limites YAML
        x_angle = self.map_normalized_to_servo(corrected_x, self.servo_neutral_x, axis='x')
        y_angle = self.map_normalized_to_servo(y_command, self.servo_neutral_y, axis='y')

        self.get_logger().debug(
            f"[DECIDE] Joystick X:{self.joystick_x:.3f} Y:{self.joystick_y:.3f} "
            f"→ Servo X:{x_angle} Y:{y_angle}"
        )

        # Publication commande finale
        self.publish_servo_command(x_angle, y_angle, source="joystick_or_avoidance")


    # =====================================================
    # OUTILS DE CONVERSION NORMALISÉ → SERVO AVEC LIMITES
    # =====================================================
    def map_normalized_to_servo(self, value, neutral, axis='x'):
        """
        Convertit une valeur [-1,1] → angle servo
        value : float [-1,1] 
        neutral : angle neutre du servo
        """
        amplitude = self.servo_amplitude

        # Clamp pour sécurité
        value_clamped = max(-1.0, min(1.0, value))

        angle = neutral + value_clamped * amplitude

        # Clamp final pour limites servo
        min_angle = neutral - amplitude
        max_angle = neutral + amplitude
        angle_clamped = int(max(min_angle, min(max_angle, angle)))

        return angle_clamped



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
