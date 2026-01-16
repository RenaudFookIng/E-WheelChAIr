# E-WheelChAIr Launch Files Guide

This guide explains the different launch file options available in the E-WheelChAIr project.

## Available Launch Files

### 1. Complete System Launch

**File:** `src/e_wheelchair_launch/e_wheelchair_launch/launch/ewheelchair_all.launch.py`

**Centralized Location:** All launch files are now located in the `e_wheelchair_launch` package for easier management.

**Description:** Launches the complete E-WheelChAIr system including all sensors and processing nodes.

**Usage:**
```bash
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

**Nodes launched:**
- Wyes Teleop Node (keyboard control)
- Servo Controller Node
- Master Node (sensor fusion and control)
- Arduino Data Receiver Node
- Depth Processing Node
- Wide Processing Node

### 2. Servo Controller Launch

**File:** `src/e_wheelchair_launch/e_wheelchair_launch/launch/servo_controller.launch.py`

**Description:** Launches the servo controller node for direct servo control.

**Usage:**
```bash
ros2 launch servo_controller servo_controller.launch.py \
    arduino_servo_port:=/dev/ttyACM1
```

**Nodes launched:**
- Servo Controller Node

### 3. Wide Camera Processing Launch

**File:** `src/e_wheelchair_launch/e_wheelchair_launch/launch/wide_camera.launch.py`

**Description:** Launches the wide-angle camera processing node.

**Usage:**
```bash
ros2 launch wide_processing wide_camera.launch.py
```

**Nodes launched:**
- Wide Processing Node

## Launch File Comparison

| Launch File | Scope | Nodes | Best For |
|-------------|-------|-------|----------|
| `ewheelchair_all.launch.py` | Complete system | 6 nodes | Full system testing, integration
| `servo_controller.launch.py` | Servo control | 1 node | Direct servo testing
| `wide_camera.launch.py` | Camera processing | 1 node | Visual processing

## When to Use Each Launch File

### Use `ewheelchair_all.launch.py` when:
- You need the complete system with all sensors
- Testing system integration
- Running full system demonstrations
- Need master node for advanced control logic

### Use `servo_controller.launch.py` when:
- Developing servo control features
- Testing servo behavior directly
- Debugging servo movements
- Calibrating servo positions

### Use `wide_camera.launch.py` when:
- Testing camera processing
- Debugging visual algorithms
- Developing computer vision features
- Monitoring camera data

## Command Line Arguments

### Common Arguments

| Argument | Default | Description | Applicable Launch Files |
|----------|---------|-------------|------------------------|
| `arduino_servo_port` | `/dev/ttyACM1` | Servo controller Arduino port | e_wheelchair_launch/servo_controller.launch.py |
| `use_sim_time` | `false` | Use simulation time | All |

### Example: Custom Port Configuration

```bash
ros2 launch e_wheelchair_launch servo_controller.launch.py \
    arduino_servo_port:=/dev/ttyUSB1
```

## Integration Patterns

### Pattern 1: Servo Control + Master Node

```bash
# Terminal 1: Launch servo controller
ros2 launch e_wheelchair_launch servo_controller.launch.py

# Terminal 2: Launch master node
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

**Use case:** Advanced control with safety logic from master node

### Pattern 2: Camera Processing + Master Node

```bash
# Terminal 1: Launch wide camera processing
ros2 launch e_wheelchair_launch wide_camera.launch.py

# Terminal 2: Launch master node
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

**Use case:** Visual processing with master control

### Pattern 3: Complete System with Customization

```bash
# Launch complete system
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py

# Or launch individual components
ros2 launch e_wheelchair_launch servo_controller.launch.py
ros2 launch e_wheelchair_launch wide_camera.launch.py
```

**Use case:** Full system with all components

## Monitoring and Debugging

### View All Nodes

```bash
ros2 node list
```

### Monitor Specific Topics

```bash
# Arduino sensor data
ros2 topic echo /arduino_data

# Servo commands
ros2 topic echo /servo_commands

# Camera data
ros2 topic echo /camera_data

# Wyes teleop intentions
ros2 topic echo /wyes_intent
```

### View Node Information

```bash
ros2 node info /wyes_teleop
ros2 node info /servo_controller
ros2 node info /arduino_data_receiver
ros2 node info /wide_processing
```

## Troubleshooting

### Issue: Launch file not found

**Solution:**
```bash
# Make sure you're in the correct directory
cd ~/E-WheelChAIr
source install/setup.bash

# Verify package is built
colcon build --packages-select e_wheelchair_launch servo_controller wide_processing
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

# Install Python dependencies
pip3 install pyserial pyyaml

# Rebuild workspace
colcon build
```

## Best Practices

### 1. Start with Complete System

Begin with `ewheelchair_all.launch.py` for full functionality.

### 2. Use Launch Arguments

Specify Arduino ports explicitly when needed.

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
# Use servo_controller.launch.py for servo development
ros2 launch servo_controller servo_controller.launch.py

# Make code changes
# Test servo movements directly
```

### 2. Integration Testing

```bash
# Test with complete system
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py

# Verify all components work together
```

### 3. Production Deployment

```bash
# Use complete system for production
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py

# Monitor system performance
```

## Configuration Management

### Configuration Files

- **Servo:** `src/servo_controller/config/servo_config.yaml`
- **Master:** `src/master_node/config/master_config.yaml`
- **Arduino:** `src/arduino_data_receiver/config/arduino_config.yaml`

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
6. **Individual launch files for each component**

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

- **Complete system** for full functionality with all sensors and processing
- **Servo controller** for direct servo testing and development
- **Camera processing** for visual data analysis

Choose the appropriate launch file based on your current needs and system requirements.