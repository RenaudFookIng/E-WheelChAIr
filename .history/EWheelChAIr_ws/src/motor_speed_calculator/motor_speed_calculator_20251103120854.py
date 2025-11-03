#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist

class MotorSpeedCalculator(Node):
    def __init__(self):
        super().__init__('motor_speed_calculator')
        self.subscription = self.create_subscription(
            String,
            'joystick_data',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, 'motor_commands', 10)
        # Paramètres du moteur (exemple : gear ratio 16.97:1, voltage nominal 24V)
        self.gear_ratio = 16.97
        self.nominal_voltage = 24.0
        self.no_load_speed = 236.0  # RPM à 24V pour gear ratio 16.97:1

    def listener_callback(self, msg):
        try:
            x, y = map(int, msg.data.split(','))
            # Convertit la valeur du joystick (0-1023) en voltage (0-24V)
            voltage_left = (y / 1023.0) * self.nominal_voltage
            voltage_right = (x / 1023.0) * self.nominal_voltage

            # Calcule la vitesse estimée (proportionnelle au voltage)
            speed_left = (voltage_left / self.nominal_voltage) * self.no_load_speed
            speed_right = (voltage_right / self.nominal_voltage) * self.no_load_speed

            # Publie les vitesses
            twist = Twist()
            twist.linear.x = float(speed_left)
            twist.angular.z = float(speed_right)
            self.publisher.publish(twist)
            self.get_logger().info(f"Voltage (L/R): {voltage_left:.2f}V, {voltage_right:.2f}V | Speed (L/R): {speed_left:.2f} RPM, {speed_right:.2f} RPM")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
