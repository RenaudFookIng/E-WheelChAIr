#!/usr/bin/env python3
"""
Servo Controller Node for E-WheelChAIr
Receives ServoCommand messages and sends commands to Arduino servo controller
"""

import rclpy
from rclpy.node import Node
from custom_msgs.msg import ServoCommand, Joystick
import serial
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class ServoControllerNode(Node):
    def __init__(self):
        super().__init__('servo_controller_node')
        
        # Subscriber for servo commands
        self.servo_sub = self.create_subscription(
            ServoCommand, 
            '/servo_commands', 
            self.servo_command_callback, 
            10
        )
        
        # Subscriber for direct joystick commands (for compatibility)
        self.joystick_sub = self.create_subscription(
            Joystick, 
            '/joystick_input', 
            self.joystick_callback, 
            10
        )
        
        # Publisher for servo status
        self.status_pub = self.create_publisher(
            ServoCommand, 
            '/servo_status', 
            10
        )
        
        # Publisher for ultrasonic data
        self.ultrasonic_pub = self.create_publisher(
            UltrasonicArray, 
            '/ultrasonic_data', 
            10
        )
        
        # Load configuration
        self.load_configuration()
        
        # Initialize serial connection
        self.setup_serial_connection()
        
        # Internal state
        self.current_x = 90  # Neutral position
        self.current_y = 90  # Neutral position
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
            
            # Validate custom servo parameters
            self.validate_servo_parameters()
        except Exception as e:
            self.get_logger().error(f"Failed to load configuration: {e}")
            # Use default configuration with custom parameters
            self.config = {
                'servo': {
                    'port': '/dev/ttyACM1',
                    'baudrate': 115200,
                    'neutral_x': 90,
                    'neutral_y': 90,
                    'min_angle': 0,
                    'max_angle': 180,
                    'custom_neutral_x': 85,      # Custom neutral for X axis
                    'custom_neutral_y': 90,      # Custom neutral for Y axis
                    'amplitude': 15,             # ±15° amplitude
                    'use_custom_mapping': True   # Enable custom mapping
                }
            }

    def validate_servo_parameters(self):
        """Validate and set default values for servo parameters"""
        servo_config = self.config.get('servo', {})
        
        # Set default values if not present
        if 'custom_neutral_x' not in servo_config:
            servo_config['custom_neutral_x'] = 85
        if 'custom_neutral_y' not in servo_config:
            servo_config['custom_neutral_y'] = 90
        if 'amplitude' not in servo_config:
            servo_config['amplitude'] = 15
        if 'use_custom_mapping' not in servo_config:
            servo_config['use_custom_mapping'] = True
        
        self.get_logger().info(f"Servo parameters: X_neutral={servo_config['custom_neutral_x']}°, "
                             f"Y_neutral={servo_config['custom_neutral_y']}°, "
                             f"Amplitude=±{servo_config['amplitude']}°")

    def setup_serial_connection(self):
        """Initialize serial connection to unified Arduino"""
        try:
            self.serial_port = serial.Serial(
                port=self.config['servo']['port'],
                baudrate=self.config['servo']['baudrate'],
                timeout=1
            )
            self.get_logger().info(f"Unified Arduino connection on {self.config['servo']['port']}")
            
            # Move to neutral position
            self.move_to_neutral()
            
            # Start main processing loop
            self.timer = self.create_timer(0.05, self.main_loop)
        except Exception as e:
            self.get_logger().error(f"Failed to connect to unified Arduino: {e}")
            self.servo_enabled = False

    def move_to_neutral(self):
        """Move servos to neutral position"""
        self.current_x = self.config['servo']['neutral_x']
        self.current_y = self.config['servo']['neutral_y']
        self.send_servo_command(self.current_x, self.current_y)

    def servo_command_callback(self, msg):
        """Handle incoming servo command messages"""
        if not self.servo_enabled:
            return
        
        # Convert normalized values to servo angles
        x_angle = self.normalize_to_servo_angle(msg.x_normalized)
        y_angle = self.normalize_to_servo_angle(msg.y_normalized)
        
        self.current_x = x_angle
        self.current_y = y_angle
        self.send_servo_command(x_angle, y_angle)

    def joystick_callback(self, msg):
        """Handle direct joystick messages (for backward compatibility)"""
        if not self.servo_enabled:
            return
        
        # Convert joystick values to servo angles
        x_angle = self.normalize_to_servo_angle(msg.x)
        y_angle = self.normalize_to_servo_angle(msg.y)
        
        self.current_x = x_angle
        self.current_y = y_angle
        self.send_servo_command(x_angle, y_angle)

    def main_loop(self):
        """Main processing loop for unified architecture"""
        # Read and publish joystick data from Arduino
        self.publish_joystick_data()
        
        # Read and publish ultrasonic data from Arduino
        self.publish_ultrasonic_data()
        
        # Process any pending servo commands
        # (Handled by existing callbacks)

    def normalize_to_servo_angle(self, value):
        """Convert normalized value (-1.0 to 1.0) to servo angle with STRICT ±15° amplitude"""
        servo_config = self.config['servo']
        
        if servo_config.get('use_custom_mapping', False):
            # Use STRICT custom mapping with ±15° amplitude limit
            return self.apply_strict_servo_mapping(value)
        else:
            # Use standard mapping (fallback)
            min_angle = servo_config['min_angle']
            max_angle = servo_config['max_angle']
            
            # Calculate angle
            angle_range = max_angle - min_angle
            neutral = min_angle + angle_range / 2
            
            return int(neutral + value * (angle_range / 2))

    def apply_strict_servo_mapping(self, normalized_value):
        """
        Apply STRICT servo mapping with ±15° amplitude limit:
        - Matches wheelchair joystick physical limits exactly
        - X-axis: 90° neutral, range 75°-105° (±15°)
        - Y-axis: 85° neutral, range 70°-100° (±15°)
        - Input: normalized value (-1.0 to 1.0)
        - Output: servo angle with STRICT ±15° constraint
        """
        servo_config = self.config['servo']
        
        # STRICT amplitude limit - critical for wheelchair safety
        amplitude = servo_config['amplitude']  # Should be 15
        
        # Map normalized input to ROS angle (0-180) with strict limits
        # -1.0 → 75 (90 - 15)
        # 0.0 → 90 (neutral)
        # 1.0 → 105 (90 + 15)
        ros_angle = 90 + int(normalized_value * amplitude)
        
        # STRICT clamping to ensure we never exceed physical limits
        ros_angle = self.strict_constrain(ros_angle, 90 - amplitude, 90 + amplitude)
        
        self.get_logger().debug(f"Strict mapping: {normalized_value:.2f} → {ros_angle}° (±{amplitude}°)")
        
        return ros_angle

    def strict_constrain(self, value, min_val, max_val):
        """STRICT constraint with safety logging"""
        constrained = max(min_val, min(max_val, value))
        
        if value != constrained:
            self.get_logger().warn(f"Value {value}° clamped to {constrained}° (range {min_val}-{max_val}°)")
        
        return constrained

    def send_servo_command(self, x_angle, y_angle):
        """Send command to unified Arduino via USB"""
        try:
            # Clamp values to standard ROS range (0-180)
            x_angle = max(0, min(180, x_angle))
            y_angle = max(0, min(180, y_angle))
            
            command = f"SERVO,{x_angle},{y_angle}\n"
            self.serial_port.write(command.encode())
            
            # Read response from Arduino (USB communication)
            response = self.read_arduino_response()
            if response:
                self.get_logger().debug(f"Arduino response: {response.strip()}")
            
            # Publish status with actual angles (after Arduino mapping)
            status_msg = ServoCommand()
            status_msg.x_angle = x_angle
            status_msg.y_angle = y_angle
            status_msg.x_normalized = (x_angle - 90) / 90.0
            status_msg.y_normalized = (y_angle - 90) / 90.0
            self.status_pub.publish(status_msg)
            
            self.get_logger().debug(f"USB Command sent: X={x_angle}°, Y={y_angle}°")
        except Exception as e:
            self.get_logger().error(f"USB Communication error: {e}")
            self.servo_enabled = False

    def publish_joystick_data(self):
        """Read and publish joystick data from unified Arduino"""
        joystick_data = self.read_joystick_data()
        if joystick_data:
            x_angle, y_angle = joystick_data
            
            # Create and publish Joystick message
            joystick_msg = Joystick()
            joystick_msg.x = (x_angle - 90) / 90.0  # Convert to normalized
            joystick_msg.y = (y_angle - 90) / 90.0
            
            self.joystick_pub.publish(joystick_msg)
            self.get_logger().debug(f"Published joystick: x={joystick_msg.x:.2f}, y={joystick_msg.y:.2f}")

    def publish_ultrasonic_data(self):
        """Read and publish ultrasonic data from unified Arduino"""
        ultrasonic_data = self.read_ultrasonic_data()
        if ultrasonic_data:
            # Create and publish UltrasonicArray message
            ultrasonic_msg = UltrasonicArray()
            ultrasonic_msg.distances = ultrasonic_data
            ultrasonic_msg.timestamp = self.get_clock().now().seconds
            
            self.ultrasonic_pub.publish(ultrasonic_msg)
            self.get_logger().debug(f"Published ultrasonic: {ultrasonic_data}")

    def read_arduino_response(self):
        """Read response from Arduino via USB"""
        try:
            # Wait a bit for response
            self.serial_port.timeout = 0.1
            if self.serial_port.inWaiting() > 0:
                return self.serial_port.readline().decode('utf-8').strip()
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading Arduino response: {e}")
            return None
        finally:
            # Restore original timeout
            self.serial_port.timeout = 1

    def read_joystick_data(self):
        """Read joystick data from unified Arduino"""
        try:
            self.serial_port.timeout = 0.1
            if self.serial_port.inWaiting() > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                if response.startswith("JOYSTICK,"):
                    return self.parse_joystick_data(response)
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading joystick data: {e}")
            return None
        finally:
            self.serial_port.timeout = 1

    def parse_joystick_data(self, data):
        """Parse JOYSTICK,X,Y message from Arduino"""
        try:
            parts = data.split(',')
            if len(parts) == 3:
                x_angle = int(parts[1])
                y_angle = int(parts[2])
                return (x_angle, y_angle)
        except Exception as e:
            self.get_logger().error(f"Invalid joystick data: {data}")
            return None
        return None

    def read_ultrasonic_data(self):
        """Read ultrasonic data from unified Arduino"""
        try:
            self.serial_port.timeout = 0.1
            if self.serial_port.inWaiting() > 0:
                response = self.serial_port.readline().decode('utf-8').strip()
                if response.startswith("ULTRASONIC,"):
                    return self.parse_ultrasonic_data(response)
            return None
        except Exception as e:
            self.get_logger().error(f"Error reading ultrasonic data: {e}")
            return None
        finally:
            self.serial_port.timeout = 1

    def parse_ultrasonic_data(self, data):
        """Parse ULTRASONIC,d1,d2,d3,d4 message from Arduino"""
        try:
            parts = data.split(',')
            if len(parts) == 5:  # ULTRASONIC + 4 distances
                distances = [float(parts[i]) for i in range(1, 5)]
                return distances
        except Exception as e:
            self.get_logger().error(f"Invalid ultrasonic data: {data}")
            return None
        return None

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            # Move to neutral position before closing
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