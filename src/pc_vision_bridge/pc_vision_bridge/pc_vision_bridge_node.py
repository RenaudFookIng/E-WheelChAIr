#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import socket
import threading
import time
from custom_msgs.msg import VisionObstacle

class PcVisionBridgeNode(Node):
    """
    PC VISION BRIDGE (ETHERNET CSV VERSION)
    =======================================
    - Mode: TCP Server
    - Port: 5000
    - Protocol: CSV
    """

    def __init__(self):
        # ⭐️ 关键修改：这里必须给父类传递节点名称
        super().__init__ ('pc_vision_bridge_node')
        
        self.vision_pub = self.create_publisher(VisionObstacle, '/vision_obstacle', 10)

        # 网络配置
        self.HOST = '0.0.0.0'
        self.PORT = 5000

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            self.server_socket.bind((self.HOST, self.PORT))
            self.server_socket.listen(1)
            self.get_logger().info(f"🟢 Waiting for PC connection... IP: {self.HOST} Port: {self.PORT}")
        except Exception as e:
            self.get_logger().error(f"🔴 Port bind failed: {e}")
            return

        self.thread = threading.Thread(target=self.accept_loop, daemon=True)
        self.thread.start()

    def accept_loop(self):
        while rclpy.ok():
            try:
                conn, addr = self.server_socket.accept()
                self.get_logger().info(f"✅ PC Connected: {addr}")
                self.read_loop(conn)
            except Exception as e:
                self.get_logger().error(f"Socket error: {e}")
                time.sleep(1)

    def read_loop(self, conn):
        buffer = ""
        while rclpy.ok():
            try:
                data = conn.recv(4096).decode('utf-8', errors='ignore')
                if not data:
                    self.get_logger().warn("⚠️ PC Disconnected")
                    break
                buffer += data
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        self.process_csv(line.strip())
            except Exception as e:
                self.get_logger().error(f"Read error: {e}")
                break
        conn.close()

    def process_csv(self, line):
        try:
            parts = line.split(',')
            if len(parts) >= 9:
                msg = VisionObstacle()
                msg.obstacle_type = parts[0].strip()
                msg.forward_offset = float(parts[1])
                msg.lateral_offset = float(parts[2])
                msg.angle_deg = float(parts[3])
                msg.depth_layer = int(parts[4])
                msg.depth_confidence = float(parts[5])
                msg.detection_confidence = float(parts[6])
                msg.seen_by_left_camera = bool(int(parts[7]))
                msg.seen_by_right_camera = bool(int(parts[8]))
                msg.stamp = self.get_clock().now().to_msg()
                self.vision_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"CSV Parse Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = PcVisionBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__  == '__main__ ':
    main()