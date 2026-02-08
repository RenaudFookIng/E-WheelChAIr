# E-WheelChAIr Arduino Controller

This directory contains the Arduino firmware for the E-WheelChAIr project.

## 📁 Files

- `ewheelchair_controller/ewheelchair_controller.ino` - Main Arduino sketch for Arduino Mega 2560
- `test_arduino.py` - Python test script for verifying Arduino functionality

## 🎯 Hardware Requirements

- **Arduino Mega 2560** (required for sufficient pins)
- **Joystick PS2** - Analog joystick for user input
- **2x MZ996R Servos** - For wheelchair joystick manipulation
- **4x HC-SR04 Ultrasonic Sensors** - For obstacle detection
- **6V Power Supply** - For servos (NOT from Arduino!)

## 🔌 Pin Configuration

### Analog Inputs
- **A0**: Joystick X-axis
- **A1**: Joystick Y-axis

### Digital Outputs
- **D9**: Servo Y (Vertical - Forward/Backward)
- **D10**: Servo X (Horizontal - Left/Right)
- **30**: Ultrasonic 1 Trigger (Rear Right sensor)
- **31**: Ultrasonic 1 Echo (Rear Right sensor)
- **32**: Ultrasonic 2 Trigger (Rear Central sensor)
- **33**: Ultrasonic 2 Echo (Rear Central sensor)
- **34**: Ultrasonic 3 Trigger (Rear Left sensor)
- **35**: Ultrasonic 3 Echo (Rear Left sensor)

### Power
- **5V**: Power for ultrasonic sensors and joystick
- **GND**: Common ground for all components

## 📦 Installation

### 1. Install Arduino IDE

```bash
# For Ubuntu/Debian
sudo apt install arduino

# Or download from https://www.arduino.cc/en/software
```

### 2. Install Required Libraries

The sketch uses the standard `Servo.h` library which comes with Arduino IDE.

### 3. Upload the Code

1. Open `ewheelchair_controller/ewheelchair_controller.ino` in Arduino IDE
2. Select board: **Arduino Mega 2560**
3. Select port: Check with `ls /dev/ttyACM*` on Linux or COM port on Windows
4. Click **Upload** (→)

## 🚀 Usage

### Serial Communication Protocol

The Arduino communicates at **115200 baud** using the following protocol:

#### Commands (ROS2 → Arduino)

- **`SERVO,X,Y`**: Set servo positions
  - `X`: X-axis angle (77-107°, neutral=92°)
  - `Y`: Y-axis angle (70-100°, neutral=85°)
  - Example: `SERVO,90,85` (neutral position)

- **`NEUTRAL`**: Return servos to neutral position
  - Response: `NEUTRAL_OK`

- **`STATUS`**: Request current status
  - Response: `STATUS,X,Y,neutralX,neutralY,amplitude`

#### Data (Arduino → ROS2)

- **`JOYSTICK,X,Y`**: Joystick position data
  - `X`: X-axis angle (0-180°)
  - `Y`: Y-axis angle (0-180°)
  - Sent every 50ms (20Hz)

- **`ULTRASONIC,d1,d2,d3,d4`**: Ultrasonic sensor distances
  - `d1`: Front sensor distance (meters)
  - `d2`: Rear sensor distance (meters)
  - `d3`: Left sensor distance (meters)
  - `d4`: Right sensor distance (meters)
  - Sent every 50ms (20Hz)

- **`SERVO_OK,X,Y`**: Confirmation of servo movement
  - `X`: Actual X position set
  - `Y`: Actual Y position set

- **`ERROR,message`**: Error messages

## ⚙️ Configuration

### Servo Parameters

Edit these values in the sketch if needed:

```cpp
int neutralX = 92;    // Neutral position for X servo
int neutralY = 85;    // Neutral position for Y servo
int amplitude = 15;   // ±15° amplitude limit
```

### Safety Features

- **Timeout**: Servos automatically return to neutral after 1 second without commands
- **Amplitude Limit**: Strict ±15° limit to prevent over-extension
- **Distance Clamping**: Ultrasonic readings constrained to 2cm-4m range

## 🧪 Testing

### Run the Test Script

```bash
python3 test_arduino.py
```

### Manual Testing

1. **Joystick Test**: Move joystick and check serial output
2. **Servo Test**: Send `SERVO,92,85` command
3. **Ultrasonic Test**: Place hand in front of sensors
4. **Safety Test**: Disconnect serial and verify timeout

## 🔄 ROS2 Integration

The Arduino connects to the ROS2 system via:

```
Arduino Mega (USB) → Raspberry Pi (/dev/ttyACM0)
```

The ROS2 `arduino_bridge_node` sends commands and receives sensor data.

## ⚠️ Safety Instructions

1. **Always test servos before connecting to wheelchair**
2. **Use external 6V power for servos** (NOT from Arduino)
3. **Ensure common ground between all components**
4. **Keep emergency stop accessible**
5. **Verify ±15° amplitude limits before use**

## 📊 Troubleshooting

### Common Issues

**Problem**: Servos don't move
- Check 8V power connection
- Verify GND is common
- Test with simple Arduino servo example

**Problem**: Ultrasonic always shows 4.0m
- Check Trig/Echo wiring
- Verify 5V power connection
- Test with hand at 20cm distance

**Problem**: No serial communication
- Check USB cable connection
- Verify correct port selection
- Ensure baud rate is 115200
- Close Arduino Serial Monitor before running ROS2

**Problem**: Joystick not responding
- Check A0/A1 connections
- Verify 5V power to joystick
- Test with Arduino analog read example

## 📚 References

- [Arduino Servo Library](https://www.arduino.cc/en/Reference/Servo)
- [HC-SR04 Ultrasonic Sensor](https://www.electronicwings.com/sensors-modules/hc-sr04-ultrasonic-sensor)
- [PS2 Joystick Guide](https://www.instructables.com/How-to-Use-a-Joystick-With-Arduino/)

## 🎓 Notes

- The Y-axis neutral position is **85°** (not 90°) to match wheelchair mechanics
- The X-axis neutral position is **92°** (not 90°) to match wheelchair mechanics
- Servo amplitude is **strictly limited to ±15°** for safety
- All ultrasonic sensors are read sequentially to avoid interference
- Serial communication uses newline (`\n`) as command terminator