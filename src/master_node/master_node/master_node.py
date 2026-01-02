import rclpy
from rclpy.node import Node
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from custom_msgs.msg import UltrasonicArray, Joystick, EmergencyData, ObstacleDetection
from geometry_msgs.msg import Twist

class MasterNode(Node):
    def __init__(self):
        super().__init__('master_node')

        # --- Publishers ---
        # Publish joystick commands to servo controller
        self.joystick_pub = self.create_publisher(Joystick, '/joystick_input', 10)

        # --- Subscribers ---
        # Note: Joystick data now comes directly from Arduino via serial, not ROS
        self.create_subscription(UltrasonicArray, '/ultrasonic_data', self.ultrasonic_callback, 10)
        self.create_subscription(EmergencyData, '/emergency_data', self.emergency_callback, 10)
        self.create_subscription(ObstacleDetection, '/obstacle_detection', self.obstacle_callback, 10)

        # --- Internal state ---
        self.emergency_stop = False
        self.joystick_x = 0.0
        self.joystick_y = 0.0
        self.obstacle_distance = 999.0
        self.servo_enabled = True  # Flag to enable/disable servo control

        # --- Servo Controller Communication ---
        self.setup_servo_controller()
        
        # --- Arduino Joystick Communication ---
        self.setup_arduino_joystick()

        self.get_logger().info('Master Node started and ready.')

    def setup_servo_controller(self):
        """Initialize serial communication with servo controller"""
        try:
            config_path = os.path.join(
                get_package_share_directory('master_node'),
                'config',
                'servo_config.yaml'
            )
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            self.servo_serial = serial.Serial(
                port=config['servo_controller']['port'],
                baudrate=config['servo_controller']['baudrate'],
                timeout=1
            )
            self.get_logger().info(f"Servo controller connected on {config['servo_controller']['port']}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to servo controller: {e}")
            self.servo_enabled = False

    def setup_arduino_joystick(self):
        """Initialize serial communication with Arduino joystick"""
        try:
            config_path = os.path.join(
                get_package_share_directory('master_node'),
                'config',
                'servo_config.yaml'
            )
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

            # Use different port for Arduino joystick
            joystick_port = config['servo_controller']['port'].replace('ACM1', 'ACM0')
            
            self.joystick_serial = serial.Serial(
                port=joystick_port,
                baudrate=115200,
                timeout=1
            )
            self.get_logger().info(f"Arduino joystick connected on {joystick_port}")
            
            # Start timer to read joystick data
            self.joystick_timer = self.create_timer(0.05, self.read_arduino_joystick)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino joystick: {e}")

    # === Callbacks ===
    # Note: Joystick data now comes from Arduino serial, not ROS callback
    # def joystick_callback(self, msg: Joystick):
    #     self.joystick_x = msg.x
    #     self.joystick_y = msg.y
    #     self.publish_joystick_command()

    def ultrasonic_callback(self, msg: UltrasonicArray):
        if msg.distances:
            self.obstacle_distance = min(msg.distances)

    def emergency_callback(self, msg: EmergencyData):
        self.emergency_stop = msg.stop
        if self.emergency_stop:
            self.get_logger().warn('Emergency stop activated!')
            self.publish_joystick_command()

    def obstacle_callback(self, msg: ObstacleDetection):
        # Suppose que le message contient la distance minimale détectée par vision
        self.obstacle_distance = msg.relative_distance

    # === Core logic ===
    def publish_joystick_command(self):
        """
        Publish joystick commands to servo controller
        Handles emergency stops and obstacle avoidance
        """
        if self.emergency_stop:
            # Emergency stop - move servos to neutral position
            self.get_logger().warn('Emergency stop activated! Servos moved to neutral.')
            self.send_servo_command(90, 90)  # Neutral position
        elif self.obstacle_distance < 0.3:  # Seuil de sécurité
            self.get_logger().info('Obstacle detected - stopping')
            # Stop movement but allow turning
            x_angle = self.normalize_to_servo_angle(self.joystick_x)
            y_angle = 90  # Neutral Y (no forward/backward)
            self.send_servo_command(x_angle, y_angle)
        else:
            # Normal operation - convert joystick values to servo angles
            x_angle = self.normalize_to_servo_angle(self.joystick_x)
            y_angle = self.normalize_to_servo_angle(self.joystick_y)
            self.send_servo_command(x_angle, y_angle)

    def normalize_to_servo_angle(self, value):
        """
        Convert normalized joystick value (-1.0 to 1.0) to servo angle (0-180)
        """
        # Map from [-1.0, 1.0] to [0, 180]
        return int((value + 1.0) * 90.0)

    def send_servo_command(self, x_angle, y_angle):
        """
        Send servo command to Arduino servo controller
        """
        if not self.servo_enabled or not hasattr(self, 'servo_serial'):
            return

        try:
            # Clamp values to valid range
            x_angle = max(0, min(180, x_angle))
            y_angle = max(0, min(180, y_angle))

            command = f"SERVO,{x_angle},{y_angle}\n"
            self.servo_serial.write(command.encode())
            
            # Also publish the joystick message for other nodes
            joystick_msg = Joystick()
            joystick_msg.x = self.joystick_x
            joystick_msg.y = self.joystick_y
            self.joystick_pub.publish(joystick_msg)
            
            self.get_logger().debug(f"Sent servo command: X={x_angle}°, Y={y_angle}°")
        except Exception as e:
            self.get_logger().error(f"Failed to send servo command: {e}")
            self.servo_enabled = False

    def read_arduino_joystick(self):
        """
        Read joystick data from Arduino and update internal state
        """
        if not hasattr(self, 'joystick_serial') or not self.joystick_serial.is_open:
            return

        try:
            line = self.joystick_serial.readline().decode('utf-8').strip()
            if line:
                parts = line.split(',')
                if len(parts) == 2:
                    self.joystick_x = float(parts[0].strip())
                    self.joystick_y = float(parts[1].strip())
                    self.publish_joystick_command()
        except Exception as e:
            self.get_logger().error(f"Error reading Arduino joystick: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MasterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'servo_serial'):
            node.servo_serial.close()
        if hasattr(node, 'joystick_serial'):
            node.joystick_serial.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
