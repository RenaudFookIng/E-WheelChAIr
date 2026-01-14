#!/usr/bin/env python3
"""
Teleop Joystick Node for E-WheelChAIr
Reads data from Arduino joystick and publishes to ROS
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import Joystick, ServoCommand
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class TeleopJoystickNode(Node):
    def __init__(self):
        super().__init__('teleop_joystick_node')
        
        # Publishers
        self.joystick_pub = self.create_publisher(Joystick, '/joystick_data', 10)
        self.servo_cmd_pub = self.create_publisher(ServoCommand, '/servo_commands', 10)
        
        # Load configuration
        self.load_configuration()
        
        # Initialize serial connection
        self.setup_serial_connection()
        
        # Timer for reading and publishing data
        self.timer = self.create_timer(0.05, self.read_and_publish)  # 20 Hz
        
        # Internal state
        self.current_x = 0.0
        self.current_y = 0.0
        self.joystick_enabled = True
        
        self.get_logger().info('Teleop Joystick Node started')

    def load_configuration(self):
        """Load configuration from YAML file"""
        config_path = os.path.join(
            get_package_share_directory('teleop_joystick'),
            'config',
            'joystick_config.yaml'
        )
        
        try:
            with open(config_path, 'r') as f:
                self.config = yaml.safe_load(f)
            self.get_logger().info(f"Configuration loaded from {config_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Use default configuration
            self.config = {
                'arduino': {
                    'port': '/dev/ttyACM0',
                    'baudrate': 115200,
                    'timeout': 1
                },
                'calibration': {
                    'x_min': -1.0,
                    'x_max': 1.0,
                    'y_min': -1.0,
                    'y_max': 1.0,
                    'deadzone': 0.1
                }
            }

    def setup_serial_connection(self):
        """Initialize serial connection to Arduino"""
        try:
            self.serial_port = serial.Serial(
                port=self.config['arduino']['port'],
                baudrate=self.config['arduino']['baudrate'],
                timeout=self.config['arduino']['timeout']
            )
            self.get_logger().info(f"Serial connection established on {self.config['arduino']['port']}")
        except Exception as e:
            self.get_logger().error(f"Failed to establish serial connection: {e}")
            self.joystick_enabled = False

    def read_and_publish(self):
        """Read data from Arduino and publish to ROS"""
        if not self.joystick_enabled or not self.serial_port.is_open:
            return

        try:
            line = self.serial_port.readline().decode('utf-8').strip()
            if line:
                self.process_joystick_data(line)
        except Exception as e:
            self.get_logger().error(f"Error reading from serial port: {e}")

    def process_joystick_data(self, data):
        """Process raw joystick data and publish ROS messages"""
        try:
            # Expected format: "X,Y" where X and Y are values from Arduino
            parts = data.split(',')
            if len(parts) == 2:
                x_raw = float(parts[0].strip())
                y_raw = float(parts[1].strip())
                
                # Apply calibration and deadzone
                x_calibrated = self.apply_calibration(x_raw, 'x')
                y_calibrated = self.apply_calibration(y_raw, 'y')
                
                # Update internal state
                self.current_x = x_calibrated
                self.current_y = y_calibrated
                
                # Publish Joystick message
                joystick_msg = Joystick()
                joystick_msg.x = x_calibrated
                joystick_msg.y = y_calibrated
                self.joystick_pub.publish(joystick_msg)
                
                # Publish ServoCommand message
                servo_msg = ServoCommand()
                servo_msg.x_normalized = x_calibrated
                servo_msg.y_normalized = y_calibrated
                servo_msg.x_angle = self.normalize_to_servo_angle(x_calibrated)
                servo_msg.y_angle = self.normalize_to_servo_angle(y_calibrated)
                self.servo_cmd_pub.publish(servo_msg)
                
                self.get_logger().debug(f"Published: x={x_calibrated:.2f}, y={y_calibrated:.2f}")
        except ValueError as e:
            self.get_logger().warn(f"Invalid data format: {data}")
        except Exception as e:
            self.get_logger().error(f"Error processing joystick data: {e}")

    def apply_calibration(self, value, axis):
        """Apply calibration and deadzone to joystick values"""
        # Apply deadzone
        deadzone = self.config['calibration']['deadzone']
        if abs(value) < deadzone:
            return 0.0
        
        # Scale to calibrated range
        if axis == 'x':
            min_val = self.config['calibration']['x_min']
            max_val = self.config['calibration']['x_max']
        else:  # y axis
            min_val = self.config['calibration']['y_min']
            max_val = self.config['calibration']['y_max']
        
        # Map from raw range to calibrated range
        calibrated = (value - min_val) / (max_val - min_val) * 2.0 - 1.0
        
        # Clamp to [-1.0, 1.0]
        return max(-1.0, min(1.0, calibrated))

    def normalize_to_servo_angle(self, value):
        """Convert normalized value to servo angle for status reporting"""
        # Simple conversion for status reporting
        return int((value + 1.0) * 90.0)

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopJoystickNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()