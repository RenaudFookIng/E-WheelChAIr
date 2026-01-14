# E-WheelChAIr Teleoperation Guide

This guide explains how to use the teleoperation system for controlling the wheelchair using the joystick connected to Arduino.

## System Overview

The teleoperation system consists of:

1. **Arduino Joystick**: Physical joystick connected to Arduino that reads user input
2. **Teleop Joystick Node**: ROS node that reads joystick data from Arduino and publishes to ROS topics
3. **Servo Controller Node**: ROS node that receives commands and sends them to Arduino servo controller
4. **Arduino Servo Controller**: Controls the actual servos based on ROS commands

## Launch Files

### 1. Full Teleoperation System (with visualization)

```bash
ros2 launch teleop_joystick teleop_joystick.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

This launches:
- Teleop Joystick Node
- Servo Controller Node  
- Visualization Node (for monitoring)

### 2. Minimal Teleoperation System

```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

This launches only the essential nodes:
- Teleop Joystick Node
- Servo Controller Node

## Configuration

### Port Configuration

By default, the system expects:
- Joystick Arduino on `/dev/ttyACM0`
- Servo Controller Arduino on `/dev/ttyACM1`

To find your Arduino ports:
```bash
ls /dev/tty*
```

### Servo Parameters

The system uses the following servo parameters:
- **Neutral Position**: X=90°, Y=90°
- **Custom Neutral**: X=90°, Y=85° (matches wheelchair physical constraints)
- **Amplitude**: ±15° (strict limit for safety)
- **Custom Mapping**: Enabled (ensures physical limits are respected)

## ROS Topics

### Published Topics

- `/joystick_data` (custom_msgs/Joystick): Raw joystick position data
- `/servo_commands` (custom_msgs/ServoCommand): Servo command messages
- `/servo_status` (custom_msgs/ServoCommand): Servo status feedback
- `/ultrasonic_data` (custom_msgs/UltrasonicArray): Ultrasonic sensor data

### Message Types

#### Joystick.msg
```
float32 x      # X-axis position (-1.0 to 1.0)
float32 y      # Y-axis position (-1.0 to 1.0)
bool button1   # Button 1 state
bool button2   # Button 2 state
```

#### ServoCommand.msg
```
float32 x_normalized  # Normalized X position (-1.0 to 1.0)
float32 y_normalized  # Normalized Y position (-1.0 to 1.0)
int32 x_angle         # X servo angle (0-180)
int32 y_angle         # Y servo angle (0-180)
```

## Usage Instructions

### 1. Connect Hardware
- Connect Joystick Arduino to USB port
- Connect Servo Controller Arduino to USB port
- Ensure servos are properly connected to servo controller

### 2. Check Permissions
```bash
sudo usermod -a -G dialout $USER
```
(You may need to logout and login again for changes to take effect)

### 3. Launch Teleoperation
```bash
ros2 launch teleop_joystick teleop_minimal.launch.py
```

### 4. Monitor System
```bash
# In a new terminal
ros2 topic echo /joystick_data
ros2 topic echo /servo_status
```

### 5. Control Wheelchair
- Move joystick to control wheelchair direction
- Joystick returns to center when released (neutral position)
- System automatically limits movement to safe ±15° amplitude

## Safety Features

1. **Strict Amplitude Limiting**: ±15° hard limit to prevent excessive movement
2. **Neutral Position**: Automatic return to neutral when joystick is centered
3. **Deadzone**: Small movements around center are ignored
4. **Error Handling**: Automatic reconnection if serial communication fails
5. **Safety Timeout**: System returns to neutral if communication is lost

## Troubleshooting

### Common Issues

#### 1. Port Not Found
```
Error: Failed to establish serial connection
```

**Solution:**
- Check Arduino connections
- Verify port names: `ls /dev/tty*`
- Update launch file with correct port names

#### 2. Permission Denied
```
Error: Permission denied opening serial port
```

**Solution:**
```bash
sudo usermod -a -G dialout $USER
```
Then logout and login again

#### 3. No Joystick Data
```
Warning: No data received from joystick
```

**Solution:**
- Check Arduino joystick wiring
- Verify Arduino code is running
- Check baud rate (should be 115200)

#### 4. Servos Not Moving
```
Error: Failed to send servo commands
```

**Solution:**
- Check servo controller Arduino connection
- Verify servo power supply
- Check servo wiring
- Test servos individually

## Advanced Configuration

### Custom Servo Mapping

To modify servo parameters, edit the servo configuration:

```yaml
# src/servo_controller/config/servo_config.yaml
servo:
  custom_neutral_x: 90      # Custom neutral for X axis
  custom_neutral_y: 85      # Custom neutral for Y axis
  amplitude: 15             # ±15° amplitude limit
  use_custom_mapping: true  # Enable custom mapping
```

### Joystick Calibration

To calibrate the joystick, modify the joystick configuration:

```yaml
# src/teleop_joystick/config/joystick_config.yaml
calibration:
  x_min: -1.0              # Minimum X value
  x_max: 1.0               # Maximum X value
  y_min: -1.0              # Minimum Y value
  y_max: 1.0               # Maximum Y value
  deadzone: 0.1            # Deadzone to ignore small movements
```

## Integration with Other Systems

The teleoperation system can be integrated with other E-WheelChAIr components:

### Master Node Integration
The master node can subscribe to `/servo_commands` and apply additional safety logic before sending to servos.

### Visualization
The visualization node can display real-time joystick and servo status:
```bash
ros2 run visualization real_time_plot
```

### Data Logging
To log teleoperation data:
```bash
ros2 bag record /joystick_data /servo_status /ultrasonic_data
```

## Emergency Procedures

### Emergency Stop
1. Release joystick (returns to neutral position)
2. Press emergency button if available
3. Disconnect power if necessary

### Manual Override
If ROS system fails, you can manually control servos using Arduino serial monitor.

## Maintenance

### Regular Checks
- Verify all connections are secure
- Check servo movement range
- Test emergency stop functionality
- Calibrate joystick periodically

### Software Updates
```bash
cd ~/E-WheelChAIr
git pull origin main
colcon build --packages-select teleop_joystick servo_controller
source install/setup.bash
```