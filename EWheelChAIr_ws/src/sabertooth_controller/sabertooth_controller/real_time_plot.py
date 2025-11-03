#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import matplotlib.pyplot as plt
from collections import deque
import numpy as np

class RealTimePlot(Node):
    def __init__(self):
        super().__init__('real_time_plot')
        # Abonnements
        self.joystick_x_sub = self.create_subscription(Float32, 'joystick_x', self.joystick_x_callback, 10)
        self.joystick_y_sub = self.create_subscription(Float32, 'joystick_y', self.joystick_y_callback, 10)
        self.voltage_left_sub = self.create_subscription(Float32, 'voltage_left', self.voltage_left_callback, 10)
        self.voltage_right_sub = self.create_subscription(Float32, 'voltage_right', self.voltage_right_callback, 10)
        self.speed_left_sub = self.create_subscription(Float32, 'speed_left', self.speed_left_callback, 10)
        self.speed_right_sub = self.create_subscription(Float32, 'speed_right', self.speed_right_callback, 10)
        # Stockage des données
        self.max_points = 100
        self.x_data = deque(maxlen=self.max_points)
        self.y_data = deque(maxlen=self.max_points)
        self.voltage_left_data = deque(maxlen=self.max_points)
        self.voltage_right_data = deque(maxlen=self.max_points)
        self.speed_left_data = deque(maxlen=self.max_points)
        self.speed_right_data = deque(maxlen=self.max_points)
        # Initialisation du graphique
        plt.ion()
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        self.line_joystick_x, = self.ax1.plot([], [], 'r-', label='Joystick X')
        self.line_joystick_y, = self.ax1.plot([], [], 'b-', label='Joystick Y')
        self.line_voltage_left, = self.ax2.plot([], [], 'g-', label='Voltage Left')
        self.line_voltage_right, = self.ax2.plot([], [], 'm-', label='Voltage Right')
        self.line_speed_left, = self.ax3.plot([], [], 'c-', label='Speed Left')
        self.line_speed_right, = self.ax3.plot([], [], 'y-', label='Speed Right')
        self.ax1.set_ylim(0, 1023)
        self.ax2.set_ylim(0, 24)
        self.ax3.set_ylim(0, 250)
        self.ax1.legend()
        self.ax2.legend()
        self.ax3.legend()
        self.ax1.set_title('Joystick Values')
        self.ax2.set_title('Voltage (V)')
        self.ax3.set_title('Speed (RPM)')

    def joystick_x_callback(self, msg):
        self.x_data.append(msg.data)
        self.update_plot()

    def joystick_y_callback(self, msg):
        self.y_data.append(msg.data)
        self.update_plot()

    def voltage_left_callback(self, msg):
        self.voltage_left_data.append(msg.data)
        self.update_plot()

    def voltage_right_callback(self, msg):
        self.voltage_right_data.append(msg.data)
        self.update_plot()

    def speed_left_callback(self, msg):
        self.speed_left_data.append(msg.data)
        self.update_plot()

    def speed_right_callback(self, msg):
        self.speed_right_data.append(msg.data)
        self.update_plot()

    def update_plot(self):
        # Mise à jour des données
        x = np.arange(len(self.x_data))
        self.line_joystick_x.set_data(x, self.x_data)
        self.line_joystick_y.set_data(x, self.y_data)
        self.line_voltage_left.set_data(x, self.voltage_left_data)
        self.line_voltage_right.set_data(x, self.voltage_right_data)
        self.line_speed_left.set_data(x, self.speed_left_data)
        self.line_speed_right.set_data(x, self.speed_right_data)
        # Ajuste les limites des axes
        if len(self.x_data) > 0:
            self.ax1.set_xlim(0, len(self.x_data))
            self.ax2.set_xlim(0, len(self.x_data))
            self.ax3.set_xlim(0, len(self.x_data))
        # Redessine le graphique
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    node = RealTimePlot()
    rclpy.spin(node)
    plt.ioff()
    plt.show()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
