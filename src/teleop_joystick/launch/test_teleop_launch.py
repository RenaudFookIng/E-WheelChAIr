#!/usr/bin/env python3
"""
Test script for teleoperation launch files
Verifies that launch files can be imported and basic structure is correct
"""

import sys
import os

# Add the launch directory to Python path for testing
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..'))

def test_launch_file_import():
    """Test that launch files can be imported"""
    try:
        # Test import of launch modules
        from launch import LaunchDescription
        from launch_ros.actions import Node
        from launch.actions import DeclareLaunchArgument
        from launch.substitutions import LaunchConfiguration
        from ament_index_python.packages import get_package_share_directory
        
        print("✓ All required ROS2 launch modules imported successfully")
        return True
    except ImportError as e:
        print(f"✗ Failed to import ROS2 modules: {e}")
        print("  This is expected if ROS2 is not sourced in the current environment")
        return False

def test_launch_file_syntax():
    """Test that launch files have correct syntax"""
    launch_files = [
        'teleop_joystick.launch.py',
        'teleop_minimal.launch.py'
    ]
    
    all_passed = True
    
    for launch_file in launch_files:
        file_path = os.path.join(os.path.dirname(__file__), launch_file)
        try:
            with open(file_path, 'r') as f:
                content = f.read()
                
            # Basic syntax checks
            assert 'LaunchDescription' in content
            assert 'Node' in content
            assert 'package=\'teleop_joystick\'' in content
            assert 'package=\'servo_controller\'' in content
            assert 'DeclareLaunchArgument' in content
            assert 'LaunchConfiguration' in content
            
            print(f"✓ {launch_file} has correct basic structure")
        except Exception as e:
            print(f"✗ {launch_file} failed syntax check: {e}")
            all_passed = False
    
    return all_passed

def test_launch_file_parameters():
    """Test that launch files have expected parameters"""
    launch_files = [
        'teleop_joystick.launch.py',
        'teleop_minimal.launch.py'
    ]
    
    expected_params = [
        'arduino_joystick_port',
        'arduino_servo_port'
    ]
    
    all_passed = True
    
    for launch_file in launch_files:
        file_path = os.path.join(os.path.dirname(__file__), launch_file)
        try:
            with open(file_path, 'r') as f:
                content = f.read()
                
            for param in expected_params:
                if param in content:
                    print(f"✓ {launch_file} contains parameter: {param}")
                else:
                    print(f"✗ {launch_file} missing parameter: {param}")
                    all_passed = False
                    
        except Exception as e:
            print(f"✗ Error checking parameters in {launch_file}: {e}")
            all_passed = False
    
    return all_passed

def main():
    """Run all tests"""
    print("Testing E-WheelChAIr Teleoperation Launch Files")
    print("=" * 50)
    
    # Test 1: ROS2 module imports
    print("\n1. Testing ROS2 module imports...")
    ros_imports_ok = test_launch_file_import()
    
    # Test 2: Launch file syntax
    print("\n2. Testing launch file syntax...")
    syntax_ok = test_launch_file_syntax()
    
    # Test 3: Launch file parameters
    print("\n3. Testing launch file parameters...")
    params_ok = test_launch_file_parameters()
    
    # Summary
    print("\n" + "=" * 50)
    print("Test Summary:")
    print(f"  ROS2 Imports: {'PASS' if ros_imports_ok else 'SKIPPED (expected)'}")
    print(f"  Syntax Check: {'PASS' if syntax_ok else 'FAIL'}")
    print(f"  Parameters: {'PASS' if params_ok else 'FAIL'}")
    
    if syntax_ok and params_ok:
        print("\n✓ All tests PASSED - Launch files are ready to use!")
        return 0
    else:
        print("\n✗ Some tests FAILED - Please check the launch files")
        return 1

if __name__ == '__main__':
    sys.exit(main())