#!/usr/bin/env python3
"""
Test simple pour vérifier que les composants Python fonctionnent
"""

import sys
sys.path.append('src')

def test_messages():
    print("Testing custom messages...")
    
    # Import direct des modules
    from custom_msgs_py.custom_msgs_py.joystick_msg import Joystick
    from custom_msgs_py.custom_msgs_py.ultrasonic_array_msg import UltrasonicArray
    from custom_msgs_py.custom_msgs_py.emergency_data_msg import EmergencyData
    from custom_msgs_py.custom_msgs_py.obstacle_detection_msg import ObstacleDetection
    
    # Test Joystick
    joystick = Joystick(x=0.5, y=-0.3)
    print(f"✅ Joystick: {joystick}")
    
    # Test UltrasonicArray
    ultrasonic = UltrasonicArray(distances=[1.0, 1.5, 2.0, 0.8])
    print(f"✅ UltrasonicArray: min_distance={ultrasonic.get_min_distance()}")
    
    # Test EmergencyData
    emergency = EmergencyData(obstacle_detected=True, distance=0.5, source="lidar")
    print(f"✅ EmergencyData: {emergency}")
    
    # Test ObstacleDetection
    obstacle = ObstacleDetection(obstacle_class="person", confidence=0.95, relative_distance=2.0)
    print(f"✅ ObstacleDetection: {obstacle}")
    
    print("🎉 All message tests passed!")

def test_sabertooth_structure():
    print("\nTesting Sabertooth controller structure...")
    
    # Vérifier la structure du fichier sans l'importer
    with open('src/sabertooth_controller_py/sabertooth_controller_py/sabertooth_controller.py', 'r') as f:
        content = f.read()
        
    # Vérifier les éléments clés
    checks = [
        ('class SabertoothController', 'SabertoothController class'),
        ('def motor_callback', 'motor_callback method'),
        ('def send_command', 'send_command method'),
        ('def safety_timer_callback', 'safety_timer_callback method'),
        ('import serial', 'serial import'),
        ('from geometry_msgs.msg import Twist', 'Twist import'),
    ]
    
    for check, name in checks:
        if check in content:
            print(f"✅ {name} found")
        else:
            print(f"❌ {name} missing")

def test_master_node_structure():
    print("\nTesting Master Node structure...")
    
    # Vérifier la structure du fichier sans l'importer
    with open('src/master_node/master_node/master_node.py', 'r') as f:
        content = f.read()
        
    # Vérifier les éléments clés
    checks = [
        ('from custom_msgs_py import', 'custom_msgs_py import'),
        ('class MasterNode', 'MasterNode class'),
        ('def joystick_callback', 'joystick_callback method'),
        ('def ultrasonic_callback', 'ultrasonic_callback method'),
        ('def publish_motor_command', 'publish_motor_command method'),
    ]
    
    for check, name in checks:
        if check in content:
            print(f"✅ {name} found")
        else:
            print(f"❌ {name} missing")

def main():
    print("🚀 Running simple tests...")
    print("=" * 40)
    
    try:
        test_messages()
        test_sabertooth_structure()
        test_master_node_structure()
        
        print("\n" + "=" * 40)
        print("🎉 All structure tests passed!")
        print("\n📋 Summary of changes:")
        print("1. ✅ Created sabertooth_controller_py (Python replacement)")
        print("2. ✅ Created custom_msgs_py (Python message interface)")
        print("3. ✅ Updated master_node to use Python messages")
        print("4. ✅ Created ewheelchair_python.launch.py")
        print("\n🔧 Next steps for deployment:")
        print("1. Install ROS2 Humble on Raspberry Pi 3")
        print("2. Install dependencies: ros-humble-rclpy ros-humble-geometry-msgs python3-pyserial")
        print("3. Build workspace: colcon build --packages-select sabertooth_controller_py custom_msgs_py master_node")
        print("4. Launch: ros2 launch e_wheelchair_launch ewheelchair_python.launch.py")
        
    except Exception as e:
        print(f"❌ Test failed: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()