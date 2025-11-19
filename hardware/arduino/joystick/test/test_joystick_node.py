import unittest
from joystick.joystick_node import JoystickNode
import rclpy

class TestJoystickNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_node_initialization(self):
        node = JoystickNode()
        self.assertEqual(node.get_name(), "joystick_node")
        node.destroy_node()

if __name__ == "__main__":
    unittest.main()
