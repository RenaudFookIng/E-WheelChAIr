# E-WheelChAIr Teleoperation Launch Files

This directory contains launch files for the E-WheelChAIr teleoperation system.

## Available Launch Files

### 1. `teleop_joystick.launch.py`
**Full teleoperation system with visualization**

Launches:
- Teleop Joystick Node (reads from Arduino joystick)
- Servo Controller Node (controls servos via Arduino)
- Visualization Node (for monitoring joystick and servo status)

**Usage:**
```bash
ros2 launch teleop_joystick teleop_joystick.launch.py 
    arduino_joystick_port:=/dev/ttyACM0 
    arduino_servo_port:=/dev/ttyACM1
```

### 2. `teleop_minimal.launch.py`
**Minimal teleoperation system**

Launches only the essential nodes:
- Teleop Joystick Node
- Servo Controller Node

**Usage:**
```bash
ros2 launch teleop_joystick teleop_minimal.launch.py 
    arduino_joystick_port:=/dev/ttyACM0 
    arduino_servo_port:=/dev/ttyACM1
```

## Configuration

### Port Configuration
- `arduino_joystick_port`: Serial port for Arduino joystick (default: `/dev/ttyACM0`)
- `arduino_servo_port`: Serial port for Arduino servo controller (default: `/dev/ttyACM1`)

### Servo Parameters
The launch files use the following servo parameters by default:
- Neutral X: 90°
- Neutral Y: 90°
- Custom Neutral X: 90°
- Custom Neutral Y: 85°
- Amplitude: ±15°
- Custom mapping: Enabled

These can be overridden by modifying the launch file parameters or the servo configuration file.

## System Architecture

```
Arduino Joystick → Teleop Joystick Node → ROS Topics → Servo Controller Node → Arduino Servo Controller → Servos
```

### ROS Topics
- `/joystick_data`: Joystick position data (custom_msgs/Joystick)
- `/servo_commands`: Servo command messages (custom_msgs/ServoCommand)
- `/servo_status`: Servo status feedback (custom_msgs/ServoCommand)
- `/ultrasonic_data`: Ultrasonic sensor data (custom_msgs/UltrasonicArray)

## Troubleshooting

### Common Issues
1. **Port not found**: Verify Arduino devices are connected and check port names with `ls /dev/tty*`
2. **Permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
3. **Baud rate mismatch**: Ensure Arduino and launch file use same baud rate (115200)

### Debugging
- Use `ros2 topic echo /joystick_data` to monitor joystick input
- Use `ros2 topic echo /servo_status` to monitor servo commands
- Check node logs with `ros2 node info` and `ros2 node list`