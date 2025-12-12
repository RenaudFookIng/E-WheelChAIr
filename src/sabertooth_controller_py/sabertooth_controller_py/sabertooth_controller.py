#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SabertoothController(Node):
    def __init__(self):
        super().__init__('sabertooth_controller_py')
        
        # Configuration du port série (configurable via paramètres ROS)
        self.declare_parameter('serial_port', '/dev/ttyS0')
        self.declare_parameter('baud_rate', 9600)
        
        self.serial_port = self.get_parameter('serial_port').get_parameter_value().string_value
        self.baud_rate = self.get_parameter('baud_rate').get_parameter_value().integer_value
        self.timeout = 1.0
        
        # Paramètres de sécurité
        self.safety_timeout = 0.5  # secondes
        self.last_command_time = time.time()
        self.serial_connection = None
        
        # Configuration QoS pour une communication fiable
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Abonnement aux commandes moteur
        self.motor_sub = self.create_subscription(
            Twist,
            '/cmd_vel',  # Topic utilisé par master_node
            self.motor_callback,
            qos_profile
        )
        
        # Timer de sécurité
        self.safety_timer = self.create_timer(
            self.safety_timeout,
            self.safety_timer_callback
        )
        
        # Initialisation de la connexion série
        self.init_serial_connection()
        
        self.get_logger().info(f'Sabertooth Controller Python node started')
        self.get_logger().info(f'Using serial port: {self.serial_port} at {self.baud_rate} baud')
    
    def init_serial_connection(self):
        """Initialise la connexion série avec le contrôleur Sabertooth"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.get_logger().info('Serial connection established successfully')
            return True
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port {self.serial_port}: {str(e)}')
            self.get_logger().error('Sabertooth controller will not function without serial connection')
            return False
    
    def motor_callback(self, msg):
        """Callback pour les commandes moteur"""
        self.last_command_time = time.time()
        
        if self.serial_connection is None or not self.serial_connection.is_open:
            self.get_logger().warn('Serial port not available, cannot send motor commands')
            return
        
        try:
            # Convertir les commandes Twist en commandes Sabertooth
            # Sabertooth utilise une plage de 0-127 pour chaque moteur
            
            # Commande linéaire (avant/arrière) - moteur 1
            motor1 = int(msg.linear.x * 127.0)
            motor1 = max(-127, min(127, motor1))  # Limiter à -127..127
            
            # Commande angulaire (rotation) - moteur 2
            motor2 = int(msg.angular.z * 127.0)
            motor2 = max(-127, min(127, motor2))  # Limiter à -127..127
            
            # Envoyer la commande au Sabertooth
            self.send_command(motor1, motor2)
            
            # Log des commandes (pour le débogage)
            if abs(motor1) > 5 or abs(motor2) > 5:  # Seuil pour éviter le spam
                self.get_logger().debug(f'Sending motor commands: M1={motor1}, M2={motor2}')
                
        except Exception as e:
            self.get_logger().error(f'Error processing motor command: {str(e)}')
    
    def send_command(self, motor1, motor2):
        """Envoie une commande aux moteurs via le protocole Sabertooth"""
        try:
            # Protocole Sabertooth 2x32 : 
            # Byte 1: Adresse (0 pour diffusion, 128 pour moteur 1, 129 pour moteur 2)
            # Byte 2: Commande moteur 1 (0-127 avant, 128-255 arrière, 64 arrêt)
            # Byte 3: Commande moteur 2 (même format)
            
            # Convertir les valeurs signées (-127..127) en format Sabertooth (0..255)
            cmd1 = self._convert_to_sabertooth_format(motor1)
            cmd2 = self._convert_to_sabertooth_format(motor2)
            
            # Créer la commande (format simplifié pour 2x32)
            command = bytes([128, cmd1, cmd2])  # 128 = adresse pour moteur 1
            
            # Envoyer la commande
            self.serial_connection.write(command)
            
        except serial.SerialException as e:
            self.get_logger().error(f'Serial communication error: {str(e)}')
            self.serial_connection.close()
            self.serial_connection = None
    
    def _convert_to_sabertooth_format(self, value):
        """Convertit une valeur signée (-127..127) en format Sabertooth (0..255)"""
        # Sabertooth utilise :
        # 0-127 : avant (0 = plein avant, 64 = arrêt, 127 = lent avant)
        # 128-255 : arrière (128 = plein arrière, 192 = arrêt, 255 = lent arrière)
        
        if value > 0:
            # Avant : 64 - value (64=arrêt, 0=plein avant)
            return 64 - int(value * 0.5)
        elif value < 0:
            # Arrière : 192 - value (192=arrêt, 255=plein arrière)
            return 192 - int(value * 0.5)
        else:
            # Arrêt
            return 64
    
    def safety_timer_callback(self):
        """Callback de sécurité pour arrêter les moteurs en cas de perte de communication"""
        current_time = time.time()
        
        if current_time - self.last_command_time > self.safety_timeout * 2:
            self.get_logger().warn('Safety timeout: No motor commands received, stopping motors')
            self.send_command(0, 0)  # Arrêter les moteurs
    
    def destroy_node(self):
        """Nettoyage lors de l'arrêt du nœud"""
        if self.serial_connection and self.serial_connection.is_open:
            try:
                self.send_command(0, 0)  # Arrêter les moteurs
                self.serial_connection.close()
                self.get_logger().info('Serial connection closed')
            except Exception as e:
                self.get_logger().error(f'Error closing serial connection: {str(e)}')
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = SabertoothController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()