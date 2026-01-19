#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import json
import threading
import glob
from builtin_interfaces.msg import Time
from custom_msgs.msg import VisionObstacle

class PcVisionBridgeNode(Node):
    """
    PC VISION BRIDGE
    =================
    - Lit les données du PC via USB (YOLO + Depth Anything)
    - Transforme en messages VisionObstacle
    - Publie sur /vision_obstacle pour le Master Node
    """

    def __init__(self):
        super().__init__('pc_vision_bridge_node')

        # Publisher vers le master_node
        self.vision_pub = self.create_publisher(
            VisionObstacle,
            '/vision_obstacle',
            10
        )

        # 🔌 Détection automatique du port USB
        ports = glob.glob('/dev/serial/by-id/*')
        if not ports:
            self.get_logger().error("Aucun PC vision détecté en USB")
            return

        self.port = ports[0]
        self.get_logger().info(f"PC vision détecté sur {self.port}")

        # Configuration du port série
        self.serial = serial.Serial(
            self.port,
            115200,
            timeout=1
        )

        # Thread de lecture
        self.thread = threading.Thread(
            target=self.read_loop,
            daemon=True
        )
        self.thread.start()

    def read_loop(self):
        """
        Boucle infinie de lecture USB
        """
        while rclpy.ok():
            try:
                line = self.serial.readline().decode(errors='ignore').strip()
                if not line:
                    continue

                data = json.loads(line)
                msg_type = data.get("type")

                if msg_type == "vision_obstacle":
                    msg = VisionObstacle()
                    msg.obstacle_type = data.get("obstacle_type", "unknown")
                    msg.angle_deg = float(data.get("angle_deg", 0.0))
                    msg.lateral_offset = float(data.get("lateral_offset", 0.0))
                    msg.forward_offset = float(data.get("forward_offset", 1.0))
                    msg.depth_layer = int(data.get("depth_layer", 1))
                    msg.depth_confidence = float(data.get("depth_confidence", 1.0))
                    msg.detection_confidence = float(data.get("detection_confidence", 1.0))
                    msg.seen_by_left_camera = bool(data.get("seen_by_left_camera", False))
                    msg.seen_by_right_camera = bool(data.get("seen_by_right_camera", False))
                    msg.stamp = self.get_clock().now().to_msg()

                    self.vision_pub.publish(msg)

            except json.JSONDecodeError:
                self.get_logger().warn("JSON invalide reçu du PC")
            except Exception as e:
                self.get_logger().error(f"Erreur USB PC: {e}")


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


if __name__ == '__main__':
    main()
