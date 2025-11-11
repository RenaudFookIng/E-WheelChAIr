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
        # Publishers pour chaque valeur
        self.joystick_x_pub = self.create_publisher(Float32, 'joystick_x', 10)
        self.joystick_y_pub = self.create_publisher(Float32, 'joystick_y', 10)
        self.voltage_left_pub = self.create_publisher(Float32, 'voltage_left', 10)
        self.voltage_right_pub = self.create_publisher(Float32, 'voltage_right', 10)
        self.speed_left_pub = self.create_publisher(Float32, 'speed_left', 10)
        self.speed_right_pub = self.create_publisher(Float32, 'speed_right', 10)
        # Paramètres du moteur
        self.gear_ratio = 16.97
        self.nominal_voltage = 24.0
        self.no_load_speed = 236.0  # RPM à 24V pour gear ratio 16.97:1

    def listener_callback(self, msg):
        try:
            x, y = map(int, msg.data.split(','))
            # Publie les valeurs du joystick
            self.joystick_x_pub.publish(Float32(data=float(x)))
            self.joystick_y_pub.publish(Float32(data=float(y)))
            # Convertit en voltage (0-24V)
            voltage_left = (y / 1023.0) * self.nominal_voltage
            voltage_right = (x / 1023.0) * self.nominal_voltage
            # Publie les voltages
            self.voltage_left_pub.publish(Float32(data=float(voltage_left)))
            self.voltage_right_pub.publish(Float32(data=float(voltage_right)))
            # Calcule et publie les vitesses
            speed_left = (voltage_left / self.nominal_voltage) * self.no_load_speed
            speed_right = (voltage_right / self.nominal_voltage) * self.no_load_speed
            self.speed_left_pub.publish(Float32(data=float(speed_left)))
            self.speed_right_pub.publish(Float32(data=float(speed_right)))
            self.get_logger().info(f"Joystick X: {x}, Y: {y} | Voltage (L/R): {voltage_left:.2f}V, {voltage_right:.2f}V | Speed (L/R): {speed_left:.2f} RPM, {speed_right:.2f} RPM")
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedCalculator()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
