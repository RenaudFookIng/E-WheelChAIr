# E-WheelChAIr Launch Files Guide

This guide explains the different launch file options available in the E-WheelChAIr project.

## Available Launch Files

### 1. Complete System Launch

**File:** `src/e_wheelchair_launch/e_wheelchair_launch/launch/ewheelchair_all.launch.py`

**Description:** Launches the complete E-WheelChAIr system including all sensors and processing nodes.

**Usage:**
```bash
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

**Nodes launched:**
- Teleop Joystick Node
- Servo Controller Node
- Master Node (sensor fusion and control)
- LiDAR Node
- Visualization Node
- (Optional) Camera Nodes

### 2. Teleoperation System Launch (Full)

**File:** `src/teleop_joystick/launch/teleop_joystick.launch.py`

**Description:** Launches the teleoperation system with visualization for monitoring.

**Usage:**
```bash
ros2 launch teleop_joystick teleop_joystick.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

**Nodes launched:**
- Teleop Joystick Node
- Servo Controller Node
- Visualization Node

### 3. Teleoperation System Launch (Minimal)

**File:** `src/teleop_joystick/launch/teleop_minimal.launch.py`

**Description:** Launches only the essential teleoperation nodes (recommended for normal use).

**Usage:**
```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

**Nodes launched:**
- Teleop Joystick Node
- Servo Controller Node

## Launch File Comparison

| Launch File | Scope | Nodes | Best For |
|-------------|-------|-------|----------|
| `ewheelchair_all.launch.py` | Complete system | 5-7 nodes | Full system testing, integration
| `teleop_joystick.launch.py` | Teleoperation + Viz | 3 nodes | Development, monitoring
| `teleop_minimal.launch.py` | Teleoperation only | 2 nodes | Normal operation, production

## When to Use Each Launch File

### Use `ewheelchair_all.launch.py` when:
- You need the complete system with all sensors
- Testing system integration
- Running full system demonstrations
- Need master node for advanced control logic

### Use `teleop_joystick.launch.py` when:
- Developing teleoperation features
- Need visualization for debugging
- Testing joystick and servo behavior
- Monitoring system performance

### Use `teleop_minimal.launch.py` when:
- Normal daily operation
- Production use
- Resource-constrained environments
- Need minimal system overhead
- Focused teleoperation testing

## Command Line Arguments

### Common Arguments

| Argument | Default | Description | Applicable Launch Files |
|----------|---------|-------------|------------------------|
| `arduino_joystick_port` | `/dev/ttyACM0` | Joystick Arduino port | teleop_*.launch.py |
| `arduino_servo_port` | `/dev/ttyACM1` | Servo controller Arduino port | teleop_*.launch.py |
| `use_sim_time` | `false` | Use simulation time | All |

### Example: Custom Port Configuration

```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyUSB0 \
    arduino_servo_port:=/dev/ttyUSB1
```

## Integration Patterns

### Pattern 1: Teleoperation + Master Node

```bash
# Terminal 1: Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Terminal 2: Launch master node
ros2 launch master_node master_node.launch.py
```

**Use case:** Advanced control with safety logic from master node

### Pattern 2: Teleoperation + LiDAR

```bash
# Terminal 1: Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Terminal 2: Launch LiDAR
ros2 launch lidar lidar.launch.py
```

**Use case:** Obstacle detection with teleoperation

### Pattern 3: Teleoperation + Cameras

```bash
# Terminal 1: Launch teleoperation
ros2 launch teleop_joystick teleop_minimal.launch.py

# Terminal 2: Launch cameras
ros2 launch image_processing wide_cameras.launch.py
```

**Use case:** Visual assistance with teleoperation

### Pattern 4: Complete System with Customization

```bash
# Launch complete system with custom ports
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py \
    arduino_joystick_port:=/dev/ttyUSB0 \
    arduino_servo_port:=/dev/ttyUSB1
```

**Use case:** Full system with non-standard port configuration

## Monitoring and Debugging

### View All Nodes

```bash
ros2 node list
```

### Monitor Specific Topics

```bash
# Joystick data
ros2 topic echo /joystick_data

# Servo commands
ros2 topic echo /servo_commands

# Servo status
ros2 topic echo /servo_status
```

### View Node Information

```bash
ros2 node info /teleop_joystick
ros2 node info /servo_controller
```

## Troubleshooting

### Issue: Launch file not found

**Solution:**
```bash
# Make sure you're in the correct directory
cd ~/E-WheelChAIr
source install/setup.bash

# Verify package is built
colcon build --packages-select e_wheelchair_launch
```

### Issue: Port already in use

**Solution:**
```bash
# Find process using port
lsof /dev/ttyACM0

# Kill process if needed
sudo kill -9 <PID>
```

### Issue: Missing dependencies

**Solution:**
```bash
# Install missing ROS2 packages
sudo apt install ros-humble-<missing-package>

# Rebuild workspace
colcon build
```

## Best Practices

### 1. Start with Minimal Configuration

Begin with `teleop_minimal.launch.py` and add components gradually.

### 2. Use Launch Arguments

Always specify Arduino ports explicitly to avoid conflicts.

### 3. Monitor System Health

Use `ros2 topic echo` and `ros2 node info` regularly.

### 4. Test in Safe Environment

Before actual use, test in a controlled environment.

### 5. Document Custom Configurations

Keep track of custom port mappings and parameters.

## Performance Considerations

### Resource Usage

| Launch File | CPU Usage | Memory Usage | Network Bandwidth |
|-------------|-----------|--------------|-------------------|
| `teleop_minimal` | Low | Low | Low |
| `teleop_joystick` | Low-Medium | Medium | Medium |
| `ewheelchair_all` | High | High | High |

### Optimization Tips

1. **Use minimal launch** for production to reduce resource usage
2. **Disable visualization** when not needed
3. **Monitor system resources** during operation
4. **Adjust update rates** in configuration files as needed

## Safety Recommendations

### Pre-Operation Checklist

1. ✅ Verify all hardware connections
2. ✅ Check Arduino port assignments
3. ✅ Test emergency stop functionality
4. ✅ Confirm servo neutral positions
5. ✅ Monitor initial system behavior

### During Operation

1. 👀 Monitor system logs for warnings
2. 🎮 Keep hands clear of moving parts
3. 🛑 Be ready to use emergency stop
4. 📊 Check resource usage periodically
5. 🔧 Address any warnings immediately

## Development Workflow

### 1. Feature Development

```bash
# Use teleop_joystick.launch.py for development
ros2 launch teleop_joystick teleop_joystick.launch.py

# Make code changes
# Test in real-time with visualization
```

### 2. Integration Testing

```bash
# Test with complete system
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py

# Verify all components work together
```

### 3. Production Deployment

```bash
# Use minimal launch for production
ros2 launch teleop_joystick teleop_minimal.launch.py

# Monitor system performance
```

## Configuration Management

### Configuration Files

- **Joystick:** `src/teleop_joystick/config/joystick_config.yaml`
- **Servo:** `src/servo_controller/config/servo_config.yaml`
- **Master:** `src/master_node/config/master_config.yaml`

### Version Control

```bash
# Commit configuration changes
git add src/*/config/*.yaml
git commit -m "Update configuration for production"
```

## Future Enhancements

### Planned Features

1. **Launch file for simulation mode**
2. **Custom launch profiles** for different use cases
3. **Automatic port detection**
4. **Health monitoring launch file**
5. **Remote control launch options**

### Contribution Guidelines

1. Follow existing launch file patterns
2. Document new launch files thoroughly
3. Test with different configurations
4. Update this guide with new options

## Support

For issues with launch files:

1. Check this guide for common solutions
2. Review launch file documentation
3. Consult ROS2 launch documentation
4. Ask for help in project discussions

## Conclusion

The E-WheelChAIr project provides flexible launch options to suit different needs:

- **Complete system** for full functionality
- **Teleoperation + visualization** for development
- **Minimal teleoperation** for production use

Choose the appropriate launch file based on your current needs and system requirements.