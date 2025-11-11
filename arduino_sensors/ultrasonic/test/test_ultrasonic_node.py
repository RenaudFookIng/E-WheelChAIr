import unittest
from ultrasonic.ultrasonic_node import UltrasonicNode
import rclpy

class TestUltrasonicNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_initialization(self):
        node = UltrasonicNode()
        self.assertEqual(node.get_name(), "ultrasonic_node")
        node.destroy_node()

if __name__ == "__main__":
    unittest.main()
