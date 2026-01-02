#!/usr/bin/env python3
"""
Test script for E-WheelChAIr Arduino Controller
Tests serial communication with the Arduino Mega
"""

import serial
import time
import sys

def find_arduino_port():
    """Find the Arduino serial port"""
    if sys.platform.startswith('win'):
        ports = ['COM3', 'COM4', 'COM5', 'COM6']
    elif sys.platform.startswith('linux') or sys.platform.startswith('darwin'):
        ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyUSB0', '/dev/ttyUSB1']
    else:
        print("Unsupported platform")
        return None
    
    for port in ports:
        try:
            s = serial.Serial(port, 115200, timeout=1)
            s.close()
            return port
        except (OSError, serial.SerialException):
            continue
    
    return None

def test_arduino_connection():
    """Test basic Arduino connection"""
    port = find_arduino_port()
    if not port:
        print("❌ No Arduino found. Please connect Arduino and check port.")
        return False
    
    print(f"🔌 Found Arduino on port: {port}")
    
    try:
        ser = serial.Serial(port, 115200, timeout=1)
        time.sleep(2)  # Wait for Arduino to initialize
        
        # Clear any initial data
        ser.reset_input_buffer()
        
        print("📡 Testing serial communication...")
        
        # Test 1: Send STATUS command
        print("📋 Test 1: Requesting status...")
        ser.write(b"STATUS\n")
        time.sleep(0.5)
        
        response = ser.readline().decode('utf-8').strip()
        if response.startswith("STATUS,"):
            print(f"✅ Status received: {response}")
        else:
            print(f"❌ Unexpected response: {response}")
            return False
        
        # Test 2: Send NEUTRAL command
        print("🎯 Test 2: Sending NEUTRAL command...")
        ser.write(b"NEUTRAL\n")
        time.sleep(0.5)
        
        response = ser.readline().decode('utf-8').strip()
        if response == "NEUTRAL_OK":
            print("✅ Servos moved to neutral position")
        else:
            print(f"❌ Unexpected response: {response}")
            return False
        
        # Test 3: Send servo command
        print("🤖 Test 3: Sending servo command...")
        ser.write(b"SERVO,90,85\n")  # Neutral position
        time.sleep(0.5)
        
        response = ser.readline().decode('utf-8').strip()
        if response.startswith("SERVO_OK,"):
            print(f"✅ Servo command executed: {response}")
        else:
            print(f"❌ Unexpected response: {response}")
            return False
        
        # Test 4: Read sensor data
        print("📊 Test 4: Reading sensor data...")
        start_time = time.time()
        
        while time.time() - start_time < 3:  # Wait up to 3 seconds
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                if response.startswith("JOYSTICK,") or response.startswith("ULTRASONIC,"):
                    print(f"✅ Sensor data: {response}")
                    break
        
        ser.close()
        print("✅ All tests passed! Arduino is working correctly.")
        return True
        
    except Exception as e:
        print(f"❌ Error: {e}")
        return False

def main():
    print("🚀 E-WheelChAIr Arduino Test Script")
    print("=" * 40)
    
    if test_arduino_connection():
        print("\n🎉 Arduino controller is ready for use!")
        print("\nNext steps:")
        print("1. Connect joystick to A0, A1")
        print("2. Connect servos to D9, D10")
        print("3. Connect ultrasonic sensors to D2-D8, D13")
        print("4. Connect to ROS2 system")
    else:
        print("\n💥 Test failed. Please check:")
        print("- Arduino is connected via USB")
        print("- Correct port is selected")
        print("- Arduino code is uploaded")
        print("- Serial monitor is closed")

if __name__ == "__main__":
    main()