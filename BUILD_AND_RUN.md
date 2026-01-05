# E-WheelChAIr - Build and Run Instructions

## 📋 Prerequisites

- ROS2 Humble installed
- Python 3.8+
- Required dependencies: `pyserial`, `pyyaml`

## 🔧 Build Instructions

### 1. Clone the repository (if not already done)
```bash
git clone https://github.com/RenaudFookIng/E-WheelChAIr.git
cd E-WheelChAIr
```

### 2. Install dependencies
```bash
sudo apt update
sudo apt install -y python3-pip python3-colcon-common-extensions
pip3 install pyserial pyyaml
```

### 3. Build the workspace
```bash
cd ~/E-WheelChAIr
colcon build --symlink-install
source install/setup.bash
```

### 4. Build specific packages (if needed)
```bash
colcon build --symlink-install --packages-select teleop_joystick servo_controller
```

## 🚀 Run Instructions

### 1. Launch teleoperation system
```bash
ros2 launch teleop_joystick teleop_minimal.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

### 2. Launch servo controller only
```bash
ros2 launch servo_controller servo_controller.launch.py \
    arduino_servo_port:=/dev/ttyACM1
```

### 3. Launch full system with visualization
```bash
ros2 launch teleop_joystick teleop_joystick.launch.py \
    arduino_joystick_port:=/dev/ttyACM0 \
    arduino_servo_port:=/dev/ttyACM1
```

## 🔍 Troubleshooting

### Package not found errors
If you get errors about missing packages:

1. Verify the package exists:
```bash
ros2 pkg list | grep teleop_joystick
```

2. Rebuild the package:
```bash
colcon build --symlink-install --packages-select teleop_joystick
```

3. Source the setup file:
```bash
source install/setup.bash
```

### Permission errors
If you get permission errors for serial ports:

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0
sudo chmod 666 /dev/ttyACM1
```

### Missing executable errors
If you get errors about missing executables:

1. Verify the executable exists:
```bash
ls -la install/teleop_joystick/lib/teleop_joystick/
```

2. Check the setup.py file:
```bash
cat src/teleop_joystick/setup.py
```

3. Rebuild the package:
```bash
colcon build --symlink-install --packages-select teleop_joystick
```

## 📊 Package Structure

### teleop_joystick
- `teleop_joystick_node.py`: Main node for joystick teleoperation
- `launch/teleop_minimal.launch.py`: Minimal launch file
- `launch/teleop_joystick.launch.py`: Full launch file with visualization
- `config/joystick_config.yaml`: Configuration file

### servo_controller
- `servo_controller_node.py`: Main node for servo control
- `launch/servo_controller.launch.py`: Launch file for servo controller
- `config/servo_config.yaml`: Configuration file

## 🔌 ROS2 Topics

### Published Topics
- `/joystick_data`: Joystick position data
- `/servo_commands`: Servo command messages
- `/servo_status`: Servo status feedback
- `/ultrasonic_data`: Ultrasonic sensor data

### Subscribed Topics
- `/servo_commands`: Servo command messages
- `/joystick_input`: Joystick input data

## 📈 Performance Notes

- Update rate: 20Hz (50ms per cycle)
- Servo amplitude: ±15° (strict limit for safety)
- Neutral positions: X=90°, Y=85°

## 🎓 Best Practices

1. Always source the setup file before running:
```bash
source install/setup.bash
```

2. Use absolute paths for launch files

3. Verify serial port connections before launching

4. Check permissions on serial ports

5. Monitor system resources during operation

## 📚 Additional Resources

- ROS2 Documentation: https://docs.ros.org/en/humble/
- Colcon Build System: https://colcon.readthedocs.io/
- Python Serial: https://pyserial.readthedocs.io/

## 🐛 Reporting Issues

If you encounter any issues, please provide:
1. The exact error message
2. The output of `ros2 pkg list`
3. The output of `ls /dev/tty*`
4. The content of any relevant log files