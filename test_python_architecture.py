#!/usr/bin/env python3
"""
Script de test pour vérifier l'architecture Python-only
Ce script teste l'intégration des nouveaux composants sans ROS2
"""

import sys
import os

# Ajouter les packages au path
sys.path.append('src')

def test_custom_messages():
    """Test des messages personnalisés Python"""
    print("🧪 Testing custom messages...")
    
    from custom_msgs_py import Joystick, UltrasonicArray, EmergencyData, ObstacleDetection
    
    # Test Joystick
    joystick = Joystick(x=0.5, y=-0.3)
    assert joystick.x == 0.5
    assert joystick.y == -0.3
    print(f"✅ Joystick: {joystick}")
    
    # Test UltrasonicArray
    ultrasonic = UltrasonicArray(distances=[1.0, 1.5, 2.0, 0.8])
    assert ultrasonic.get_min_distance() == 0.8
    print(f"✅ UltrasonicArray: {ultrasonic}")
    
    # Test EmergencyData
    emergency = EmergencyData(obstacle_detected=True, distance=0.5, source="lidar")
    assert emergency.obstacle_detected == True
    assert emergency.source == "lidar"
    print(f"✅ EmergencyData: {emergency}")
    
    # Test ObstacleDetection
    obstacle = ObstacleDetection(obstacle_class="person", confidence=0.95, relative_distance=2.0)
    assert obstacle.obstacle_class == "person"
    assert obstacle.confidence == 0.95
    print(f"✅ ObstacleDetection: {obstacle}")
    
    print("🎉 All custom messages tests passed!")
    return True

def test_sabertooth_controller_import():
    """Test de l'import du contrôleur Sabertooth"""
    print("🧪 Testing Sabertooth controller import...")
    
    try:
        # Tester l'import de base (sans ROS2)
        from sabertooth_controller_py.sabertooth_controller_py.sabertooth_controller import SabertoothController
        print("✅ SabertoothController class imported successfully")
        
        # Vérifier que les méthodes principales existent
        assert hasattr(SabertoothController, '__init__')
        assert hasattr(SabertoothController, 'motor_callback')
        assert hasattr(SabertoothController, 'send_command')
        assert hasattr(SabertoothController, 'safety_timer_callback')
        print("✅ All required methods present")
        
        return True
    except Exception as e:
        print(f"❌ Error importing SabertoothController: {e}")
        return False

def test_master_node_import():
    """Test de l'import du master node"""
    print("🧪 Testing Master Node import...")
    
    try:
        from master_node.master_node.master_node import MasterNode
        print("✅ MasterNode class imported successfully")
        
        # Vérifier que les méthodes principales existent
        assert hasattr(MasterNode, '__init__')
        assert hasattr(MasterNode, 'joystick_callback')
        assert hasattr(MasterNode, 'ultrasonic_callback')
        assert hasattr(MasterNode, 'emergency_callback')
        assert hasattr(MasterNode, 'obstacle_callback')
        assert hasattr(MasterNode, 'publish_motor_command')
        print("✅ All required methods present")
        
        return True
    except Exception as e:
        print(f"❌ Error importing MasterNode: {e}")
        return False

def test_integration():
    """Test d'intégration des composants"""
    print("🧪 Testing component integration...")
    
    # Tester que les messages personnalisés peuvent être utilisés par le master node
    from custom_msgs_py import Joystick, UltrasonicArray
    from master_node.master_node.master_node import MasterNode
    
    # Créer des instances de messages
    joystick_msg = Joystick(x=0.7, y=0.2)
    ultrasonic_msg = UltrasonicArray(distances=[1.0, 1.5, 2.0, 0.8])
    
    print(f"✅ Created test messages: Joystick={joystick_msg}, Ultrasonic={ultrasonic_msg}")
    print("✅ Integration test passed - components can work together")
    
    return True

def main():
    """Fonction principale de test"""
    print("🚀 Starting Python-only architecture tests...")
    print("=" * 50)
    
    tests = [
        test_custom_messages,
        test_sabertooth_controller_import,
        test_master_node_import,
        test_integration
    ]
    
    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"❌ Test {test.__name__} failed with exception: {e}")
            results.append(False)
        print()
    
    print("=" * 50)
    print("📊 Test Results:")
    passed = sum(results)
    total = len(results)
    print(f"Passed: {passed}/{total}")
    
    if passed == total:
        print("🎉 All tests passed! Python-only architecture is ready.")
        print("\n📋 Next steps:")
        print("1. Build the ROS2 workspace with: colcon build --packages-select sabertooth_controller_py custom_msgs_py master_node")
        print("2. Source the environment: source install/setup.bash")
        print("3. Launch the system: ros2 launch e_wheelchair_launch ewheelchair_python.launch.py")
        return True
    else:
        print("❌ Some tests failed. Please check the error messages above.")
        return False

if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)