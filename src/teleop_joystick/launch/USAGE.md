# Usage Guide for E-WheelChAIr Teleoperation Launch Files

This guide explains how to use the teleoperation launch files in the E-WheelChAIr project.

## Quick Start

### 1. Build the packages

```bash
cd ~/E-WheelChAIr
colcon build --packages-select teleop_joystick servo_controller visualization
source install/setup.bash
```

### 2. Launch teleoperation system

```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

## Launch File Options

### Minimal Launch (Recommended)

```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

**Nodes launched:**
- `teleop_joystick` - Reads joystick data from Arduino
- `servo_controller` - Controls servos based on joystick input

### Full Launch (with Visualization)

```bash
ros2 launch teleop_joystick teleop_joystick.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

**Nodes launched:**
- `teleop_joystick` - Reads joystick data from Arduino
- `servo_controller` - Controls servos based on joystick input
- `visualization` - Real-time plotting of joystick and servo data

## Command Line Arguments

### Required Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `arduino_joystick_port` | `/dev/ttyACM0` | Serial port for joystick Arduino |
| `arduino_servo_port` | `/dev/ttyACM1` | Serial port for servo controller Arduino |

### Optional Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `use_sim_time` | `false` | Use simulation time (for testing) |

## Monitoring the System

### View ROS Topics

```bash
ros2 topic list
```

Expected topics:
- `/joystick_data` - Joystick position data
- `/servo_commands` - Servo command messages
- `/servo_status` - Servo status feedback
- `/ultrasonic_data` - Ultrasonic sensor data

### Monitor Joystick Data

```bash
ros2 topic echo /joystick_data
```

### Monitor Servo Commands

```bash
ros2 topic echo /servo_commands
```

### Monitor Servo Status

```bash
ros2 topic echo /servo_status
```

## Troubleshooting

### Find Arduino Ports

```bash
ls /dev/tty*
```

### Check Node Status

```bash
ros2 node list
ros2 node info /teleop_joystick
ros2 node info /servo_controller
```

### View Logs

```bash
ros2 node info /teleop_joystick --verbose
ros2 node info /servo_controller --verbose
```

## Integration with Other Systems

### Using with Master Node

To integrate with the master node for advanced control:

```bash
# Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Launch master node in another terminal
ros2 launch master_node master_node.launch.py
```

The master node will subscribe to `/servo_commands` and can apply additional safety logic.

### Using with LiDAR

```bash
# Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Launch LiDAR in another terminal
ros2 launch lidar lidar.launch.py
```

### Using with Camera System

```bash
# Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Launch cameras in another terminal
ros2 launch image_processing wide_cameras.launch.py
```

## Configuration Files

### Joystick Configuration

Edit `src/teleop_joystick/config/joystick_config.yaml` to modify:
- Serial port settings
- Joystick calibration
- Deadzone parameters

### Servo Configuration

Edit `src/servo_controller/config/servo_config.yaml` to modify:
- Servo neutral positions
- Amplitude limits
- Safety parameters

## Safety Notes

1. **Always test in safe environment** before actual use
2. **Keep hands clear** of moving parts during operation
3. **Monitor system logs** for any warnings or errors
4. **Use emergency stop** if unexpected behavior occurs
5. **Start with minimal launch** to reduce complexity during testing

## Example Workflow

### 1. Hardware Setup
- Connect joystick Arduino to USB port
- Connect servo controller Arduino to USB port
- Connect servos to servo controller
- Power on all systems

### 2. Software Setup
```bash
# Source ROS2 environment
source /opt/ros/humble/setup.bash
source ~/E-WheelChAIr/install/setup.bash

# Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

### 3. Monitoring
```bash
# In new terminal
ros2 topic echo /joystick_data

# In another terminal
ros2 topic echo /servo_status
```

### 4. Operation
- Move joystick gently to test servo response
- Observe system behavior and logs
- Adjust configuration as needed

### 5. Shutdown
```bash
# Press Ctrl+C in launch terminal
# Verify servos return to neutral position
# Power off systems
```

## Best Practices

1. **Start with minimal configuration** and add complexity gradually
2. **Test each component individually** before full system integration
3. **Use visualization** for debugging and monitoring
4. **Log data** during testing for analysis
5. **Document changes** to configuration files

## Common Issues and Solutions

### Issue: "Port not found"
**Solution:** Verify Arduino connections and port names with `ls /dev/tty*`

### Issue: "Permission denied"
**Solution:** Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Issue: "No data from joystick"
**Solution:** Check Arduino wiring and code, verify baud rate (115200)

### Issue: "Servos not moving"
**Solution:** Check servo connections, power supply, and Arduino code

### Issue: "Unexpected servo movement"
**Solution:** Check calibration, reduce amplitude, verify neutral positions