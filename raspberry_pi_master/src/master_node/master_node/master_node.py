import rclpy
from rclpy.node import Node

from custom_msgs.msg import UltrasonicArray, Joystick, EmergencyData, ObstacleDetection
from geometry_msgs.msg import Twist

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        # --- Publishers ---
        self.motor_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- Subscribers ---
        self.create_subscription(Joystick, '/joystick_data', self.joystick_callback, 10)
        self.create_subscription(UltrasonicArray, '/ultrasonic_data', self.ultrasonic_callback, 10)
        self.create_subscription(EmergencyData, '/emergency_data', self.emergency_callback, 10)
        self.create_subscription(ObstacleDetection, '/obstacle_detection', self.obstacle_callback, 10)

        # --- Internal state ---
        self.emergency_stop = False
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.obstacle_distance = 999.0

        self.get_logger().info('Master Node started and ready.')

    # === Callbacks ===
    def joystick_callback(self, msg: Joystick):
        self.joystick_x = msg.x
        self.joystick_y = msg.y
        self.publish_motor_command()

    def ultrasonic_callback(self, msg: UltrasonicArray):
        if msg.distances:
            self.obstacle_distance = min(msg.distances)

    def emergency_callback(self, msg: EmergencyData):
        self.emergency_stop = msg.stop
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated!')
            self.publish_motor_command()

    def obstacle_callback(self, msg: ObstacleDetection):
        # Suppose que le message contient la distance minimale détectée par vision
        self.obstacle_distance = msg.min_distance

    # === Core logic ===
    def publish_motor_command(self):
        cmd = Twist()

        if self.emergency_stop:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif self.obstacle_distance < 0.3:  # Seuil de sécurité
            self.get_logger().info('Obstacle detected - stopping')
            cmd.linear.x = 0.0
            cmd.angular.z = self.joystick_x * 0.5
        else:
            cmd.linear.x = self.joystick_y * 1.0
            cmd.angular.z = self.joystick_x * 0.7

        self.motor_pub.publish(cmd)

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
