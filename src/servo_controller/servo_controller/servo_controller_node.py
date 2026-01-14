#!/usr/bin/env python3
"""
Servo Controller Node for E-WheelChAIr
Receives ServoCommand messages from Master Node and sends commands to Arduino servo controller.
Publishes servo status.
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ServoCommand
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')

        # Subscriber for servo commands from Master Node
        self.servo_sub = self.create_subscription(
            ServoCommand,
            '/servo_commands',
            self.servo_command_callback,
            10
        )

        # Publisher for servo status
        self.status_pub = self.create_publisher(
            ServoCommand,
            '/servo_status',
            10
        )

        # Load configuration
        self.load_configuration()

        # Initialize serial connection to Arduino
        self.setup_serial_connection()

        # Internal state
        self.current_x = self.config['servo']['neutral_x']
        self.current_y = self.config['servo']['neutral_y']
        self.servo_enabled = True

        self.get_logger().info('Servo Controller Node started')

    def load_configuration(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            get_package_share_directory('servo_controller'),
            'config',
            'servo_config.yaml'
        )
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f"Configuration loaded from {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Default configuration
            self.config = {
                'servo': {
                    'port': '/dev/ttyACM1',
                    'baudrate': 115200,
                    'neutral_x': 90,
                    'neutral_y': 90,
                    'min_angle': 0,
                    'max_angle': 180,
                    'amplitude': 15,
                    'use_custom_mapping': True
                }
            }

    def setup_serial_connection(self):
        """Initialize serial connection to Arduino"""
        try:
            self.serial_port = serial.Serial(
                port=self.config['servo']['port'],
                baudrate=self.config['servo']['baudrate'],
                timeout=1
            )
            self.get_logger().info(f"Arduino connected on {self.config['servo']['port']}")

            # Move to neutral position
            self.move_to_neutral()

        except Exception as e:
            self.get_logger().error(f"Failed to connect to Arduino: {e}")
            self.servo_enabled = False

    def move_to_neutral(self):
        """Move servos to neutral position"""
        self.current_x = self.config['servo']['neutral_x']
        self.current_y = self.config['servo']['neutral_y']
        self.send_servo_command(self.current_x, self.current_y)

    def servo_command_callback(self, msg: ServoCommand):
        """Handle incoming servo commands from Master Node"""
        if not self.servo_enabled:
            return

        # Convert normalized values (-1.0 to 1.0) to servo angles
        x_angle = self.normalize_to_servo_angle(msg.x_normalized)
        y_angle = self.normalize_to_servo_angle(msg.y_normalized)

        self.current_x = x_angle
        self.current_y = y_angle

        self.send_servo_command(x_angle, y_angle)

    def normalize_to_servo_angle(self, value: float) -> int:
        """Convert normalized value (-1.0 to 1.0) to servo angle with ±amplitude"""
        servo_config = self.config['servo']
        amplitude = servo_config.get('amplitude', 15)
        neutral = servo_config.get('neutral_x') if value == 0 else servo_config.get('neutral_y')
        # Map -1..1 to ±amplitude around neutral
        angle = neutral + int(value * amplitude)
        # Clamp to 0-180°
        return max(0, min(180, angle))

    def send_servo_command(self, x_angle: int, y_angle: int):
        """Send servo command to Arduino and publish status"""
        try:
            command = f"SERVO,{x_angle},{y_angle}\n"
            self.serial_port.write(command.encode())

            # Optionally read Arduino response
            if self.serial_port.inWaiting() > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                self.get_logger().debug(f"Arduino response: {response}")

            # Publish current servo status
            status_msg = ServoCommand()
            status_msg.x_angle = x_angle
            status_msg.y_angle = y_angle
            status_msg.x_normalized = (x_angle - self.config['servo']['neutral_x']) / self.config['servo']['amplitude']
            status_msg.y_normalized = (y_angle - self.config['servo']['neutral_y']) / self.config['servo']['amplitude']
            self.status_pub.publish(status_msg)

        except Exception as e:
            self.get_logger().error(f"Error sending command to Arduino: {e}")
            self.servo_enabled = False

    def destroy_node(self):
        """Clean up resources and move to neutral before exit"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.move_to_neutral()
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ServoControllerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
