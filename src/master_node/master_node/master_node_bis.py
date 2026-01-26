import rclpy
import yaml
import os
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

pkg_share = get_package_share_directory('master_node')

master_config_file = os.path.join(
    pkg_share, 'config', 'master_config.yaml'
)
servo_config_file = os.path.join(
    pkg_share, 'config', 'servo_config.yaml'
)

from custom_msgs.msg import (
    EmergencyData,
    ObstacleDetection,   # (peut rester importé, mais plus utilisé)
    VisionObstacle,
    Joystick,
    UltrasonicArray,
    ServoCommand,
    WyesIntent
)

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

# ✅ Nouveau type pour /person_polar
from geometry_msgs.msg import PointStamped


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
        self.servo_neutral_x = servo_cfg['servo_x']['neutral']
        self.servo_neutral_y = servo_cfg['servo_y']['neutral']
        self.servo_min_x = servo_cfg['servo_x']['min']
        self.servo_max_x = servo_cfg['servo_x']['max']
        self.servo_min_y = servo_cfg['servo_y']['min']
        self.servo_max_y = servo_cfg['servo_y']['max']
        self.servo_amplitude = servo_cfg.get('amplitude', 15)

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

        self.create_subscription(
            Joystick,
            '/joystick/data',
            self.joystick_callback,
            10
        )

        self.create_subscription(
            UltrasonicArray,
            '/ultrasonic/data',
            self.ultrasonic_callback,
            10
        )

        self.create_subscription(
            VisionObstacle,
            '/vision/obstacle',
            self.vision_obstacle_callback,
            10
        )

        # ❌ Ancien depth topic
        # self.create_subscription(
        #     ObstacleDetection,
        #     '/obstacle_detection',
        #     self.front_obstacle_callback,
        #     10
        # )

        # ✅ Nouveau topic: /person_polar (PointStamped)
        self.create_subscription(
            PointStamped,
            '/person_polar',
            self.person_polar_callback,
            10
        )

        self.create_subscription(
            EmergencyData,
            '/emergency_data',
            self.emergency_callback,
            10
        )

        self.wyes_direction = "stop"
        self.create_subscription(
            WyesIntent,
            '/wyes_intent',
            self.wyes_intent_callback,
            10
        )

        self.get_logger().info("Master Node initialized with YAML configs")

        # =====================================================
        #                    PUBLISHERS
        # =====================================================
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
        if msg.x is None or msg.y is None:
            return

        self.joystick_x = float(msg.x)
        self.joystick_y = float(msg.y)

        self.decide_and_publish()

    def ultrasonic_callback(self, msg: UltrasonicArray):
        if len(msg.distances) > 0:
            self.rear_obstacle_distance = min(msg.distances)

    # ❌ Ancien callback depth
    # def front_obstacle_callback(self, msg: ObstacleDetection):
    #     self.front_obstacle_distance = msg.relative_distance

    # ✅ Nouveau callback /person_polar : on n'utilise QUE x
    def person_polar_callback(self, msg: PointStamped):
        """
        /person_polar : geometry_msgs/PointStamped
        On utilise uniquement msg.point.x comme distance frontale (m).
        """
        self.front_obstacle_distance = float(msg.point.x)

    def vision_obstacle_callback(self, msg: VisionObstacle):
        if msg.detection_confidence < 0.5:
            self.vision_obstacle = None
        else:
            self.vision_obstacle = msg
            self.front_obstacle_distance = msg.forward_offset

    def emergency_callback(self, msg: EmergencyData):
        self.emergency_stop = msg.stop

        if self.emergency_stop:
            self.get_logger().warn(" EMERGENCY STOP ACTIVATED")
            self.decide_and_publish()

    def wyes_intent_callback(self, msg: WyesIntent):
        if msg.direction:
            self.wyes_direction = msg.direction
            self.get_logger().debug(f"[WYES] Direction reçue: {self.wyes_direction}")
            self.decide_and_publish()

    # =====================================================
    #              LOGIQUE DE DÉCISION CENTRALE
    # =====================================================
    def decide_and_publish(self):
        # PRIORITÉ 1 : ARRÊT D’URGENCE
        if self.emergency_stop:
            x_angle = self.servo_neutral_x
            y_angle = self.servo_neutral_y
            self.get_logger().info("[EMERGENCY] Stop immédiat")
            self.publish_servo_command(x_angle, y_angle, source="emergency")
            return

        corrected_x = self.joystick_x
        y_command = self.joystick_y

        joystick_active = abs(self.joystick_x) > 0.01 or abs(self.joystick_y) > 0.01

        if joystick_active:
            source = "joystick"
        else:
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

        # PRIORITÉ 2 : OBSTACLE AVANT (distance front_obstacle_distance)
        if y_command > 0 and self.front_obstacle_distance < self.front_obstacle_caution_distance:
            # (on garde la logique existante liée à vision_obstacle)
            if self.vision_obstacle:
                corrected_x -= self.avoidance_gain * self.vision_obstacle.lateral_offset
                self.get_logger().debug(
                    f"[AVANT] Correction direction: {corrected_x:.3f} (obstacle lat: {self.vision_obstacle.lateral_offset:.3f})"
                )
            if self.front_obstacle_distance < self.front_obstacle_stop_distance:
                y_command = 0.0
                self.get_logger().info(
                    f"[AVANT] Blocage complet, obstacle trop proche: {self.front_obstacle_distance:.2f} m"
                )

        # PRIORITÉ 3 : OBSTACLE ARRIÈRE (⚠️ INCHANGÉ comme demandé)
        if self.joystick_y < 0 and self.rear_obstacle_distance < self.rear_obstacle_stop_distance:
            y_command = 0.0
            self.get_logger().info(
                f"[ARRIÈRE] Blocage marche arrière, obstacle à {self.rear_obstacle_distance:.2f} m"
            )

        # PRIORITÉ 4 : COMMANDE NORMALE
        x_angle = self.map_normalized_to_servo(corrected_x, self.servo_neutral_x, axis='x')
        y_angle = self.map_normalized_to_servo(y_command, self.servo_neutral_y, axis='y')

        self.get_logger().debug(
            f"[DECIDE] Joystick X:{self.joystick_x:.3f} Y:{self.joystick_y:.3f} "
            f"→ Servo X:{x_angle} Y:{y_angle}"
        )

        self.publish_servo_command(x_angle, y_angle, source="joystick_or_avoidance")

    # =====================================================
    # OUTILS DE CONVERSION NORMALISÉ → SERVO AVEC LIMITES
    # =====================================================
    def map_normalized_to_servo(self, value, neutral, axis='x'):
        amplitude = self.servo_amplitude

        # Conversion [0,1] -> [-1,1] (si besoin)
        norm = value * 2.0 - 1.0

        angle = neutral + norm * amplitude

        min_angle = neutral - amplitude
        max_angle = neutral + amplitude
        angle_clamped = int(max(min_angle, min(max_angle, angle)))

        self.get_logger().debug(
            f"[MAP] Axis:{axis} Value:{value:.3f} → Norm:{norm:.3f} → Angle:{angle_clamped} "
            f"(Neutral:{neutral}, ±Amplitude:{amplitude})"
        )

        return angle_clamped

    # =====================================================
    #           PUBLICATION COMMANDE SERVO
    # =====================================================
    def publish_servo_command(self, x_angle, y_angle, source):
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

