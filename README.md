# E-WheelChAIr
**Smart Embedded System for Electric Wheelchairs**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)

---

## Description
E-WheelChAIr is an open-source project designed to enhance autonomy and accessibility for electric wheelchairs. It integrates:
- A **ROS2-based architecture** for real-time control and sensor fusion.
- **3D-printed mechanical parts** (sensor mounts, joystick adapters).
- **Modular design** for easy customization and collaboration with partners like Sorbonne University.

---

## Project Structure
```
E-WheelChAIr/
├── hardware
│   ├── 3d_models
│   │   ├── camera_ultrasound_module
│   │   └── Joystick_mount
│   │       ├── Joystick _for_wheelchair.iges
│   │       ├── Joystick_wheelchair.step
│   │       └── thumbstick_all.iges
│   │
│   └── arduino
│       ├── joystick
│       │   ├── joystick
│       │   ├── package.xml
│       │   ├── setup.py
│       │   └── test
│       └── ultrasonic
│           ├── package.xml
│           ├── resource
│           ├── setup.py
│           ├── test
│           └── ultrasonic
|
├── resources
│   ├── docs
│   └── utils
|
├── src
|   ├── custom_msgs_py          # Python message interfaces
|   │   ├── custom_msgs_py
|   │   │   ├── __init__.py
|   │   │   ├── emergency_data_msg.py
|   │   │   ├── joystick_msg.py
|   │   │   ├── obstacle_detection_msg.py
|   │   │   └── ultrasonic_array_msg.py
|   │   ├── package.xml
|   │   ├── resource
|   │   └── setup.py
|   ├── e_wheelchair_launch
|   │   ├── e_wheelchair_launch
|   |   │   ├── __init__.py
|   |   │   └── launch
|   |   │       ├── ewheelchair_all.launch.py
|   |   │       └── ewheelchair_python.launch.py
|   │   ├── package.xml
|   │   └── setup.py
|   ├── master_node              # Central control (Python)
|   │   ├── master_node
|   │   │   ├── __init__.py
|   │   │   └── master_node.py
|   │   ├── CMakeLists.txt
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── motor_speed_calculator   # Motor speed calculation (Python)
|   │   ├── motor_speed_calculator
|   │   │   ├── __init__.py
|   │   │   └── motor_speed_calculator.py
|   │   ├── CMakeLists.txt
|   │   ├── package.xml
|   │   └── setup.py
|   ├── sabertooth_controller_py # Motor controller (Python)
|   │   ├── sabertooth_controller_py
|   │   │   └── sabertooth_controller.py
|   │   ├── package.xml
|   │   └── setup.py
|   └── visualization            # Real-time plotting (Python)
|       ├── visualization
|       │   ├── __init__.py
|       │   └── real_time_plot.py
|       ├── CMakeLists.txt
|       ├── package.xml
|       └── setup.py
|
├── LICENSE
├── README.md
└── requirements.txt           # Python dependencies
```

---

## Prerequisites

### Hardware
- Electric wheelchair with Sabertooth 2x32A motor driver (or equivalent).
- Sensors: Intel Realsense (depth camera), HC-SR04 (ultrasonic), Arduino (I/O interface).
- Joystick or alternative control input device.
- **Raspberry Pi 3** (or equivalent) running Raspberry Pi OS 64-bit.

### Software

#### For Raspberry Pi 3 (Python-only version):
- **ROS2 Humble** (recommended for Raspberry Pi 3)
- **Python 3.8+**
- **Required system packages:**
  ```bash
  sudo apt update
  sudo apt install -y python3-pip python3-colcon-common-extensions \
                     ros-humble-rclpy ros-humble-geometry-msgs \
                     python3-pyserial python3-matplotlib \
                     python3-numpy python3-opencv
  ```

#### Python Dependencies:
Install the required Python packages using pip:
```bash
pip install -r requirements.txt
```

#### For Development (Full version):
- ROS2 Humble (or Foxy).
- colcon (build tool for ROS2).
- FreeCAD 0.20+ (for 3D modeling).
- Git LFS (for versioning large 3D files).

---

## Installation and Setup

### 1. Clone the Repository
```bash
git clone https://github.com/RenaudFookIng/E-WheelChAIr.git
cd E-WheelChAIr
```

### 2. Install Python Dependencies
```bash
pip install -r requirements.txt
```

### 3. Build the ROS Workspace

#### For Raspberry Pi 3 (Python-only):
```bash
# Build only the Python packages needed for Raspberry Pi 3
colcon build --packages-select sabertooth_controller_py custom_msgs_py master_node motor_speed_calculator e_wheelchair_launch visualization
source install/setup.bash
```

#### For Full Development:
```bash
# Build all packages (including C++ ones)
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the System

#### For Raspberry Pi 3 (Python-only version):
```bash
ros2 launch e_wheelchair_launch ewheelchair_python.launch.py
```

#### For development with full features:
```bash
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

**Note:** The Python-only version avoids C++ dependencies and is specifically designed for Raspberry Pi 3 compatibility.

---

## ROS Packages

### Python-Only Packages (Raspberry Pi 3 Compatible)

This project is now **100% Python** and fully compatible with Raspberry Pi 3:

| Package                     | Description                                  | Status          |
|-----------------------------|----------------------------------------------|-----------------|
| `sabertooth_controller_py`  | Python motor controller using pyserial       | ✅ Active       |
| `custom_msgs_py`            | Python interface for custom messages         | ✅ Active       |
| `master_node`               | Central control node (Python)                | ✅ Active       |
| `motor_speed_calculator`    | Motor speed calculation (Python)             | ✅ Active       |
| `e_wheelchair_launch`       | Launch files including Python-only version   | ✅ Active       |
| `visualization`             | Real-time plotting and monitoring            | ✅ Active       |

### Key Features:
- **No C++ dependencies** - All packages are pure Python
- **Raspberry Pi 3 optimized** - Reduced memory footprint
- **Easy debugging** - Full Python stack traces and logging
- **Fast development** - No compilation required for changes

---

Pour se connecter à la plateforme :
Nom de la plateforme : ewheelchair
```bash
ssh ewheelchair@192.168.4.1
```
mdp : ewheelchair

---

## 3D Models
- **Location**: [`3d_models/`](3d_models/)
- **Formats**: STL and STEP files for 3D printing and simulation.
- **Tools**: Designed in OnShape.
- **Note**: Use `git lfs pull` after cloning to retrieve 3D files.

---

## Raspberry Pi 3 Specific Notes

### Python-Only Architecture
This project is now **100% Python** and specifically designed for Raspberry Pi 3 compatibility:

- **✅ No C++ dependencies** - All motor control and message handling in Python
- **✅ Reduced memory footprint** - Optimized for Raspberry Pi 3's 1GB RAM
- **✅ Easier debugging** - Full Python stack traces and logging
- **✅ Faster iteration** - No compilation needed for changes
- **✅ Better compatibility** - Works with Raspberry Pi OS 64-bit

### Performance Considerations
- The Python version uses `pyserial` for motor control (Sabertooth 2x32A)
- Message processing uses Python classes with ROS2 rclpy
- All safety features (emergency stop, obstacle detection) are preserved
- Real-time plotting uses matplotlib for visualization

### Known Limitations
- Advanced features (RealSense camera, LiDAR) are optional and can be disabled
- For best performance, use ROS2 Humble with Python 3.8+
- Memory usage is optimized but monitor with `htop` during operation

### Memory Optimization Tips
```bash
# Monitor memory usage
htop

# Kill unnecessary processes
sudo systemctl stop unnecessary-service

# Use lightweight desktop environment
sudo apt install lxde
```

## Python Dependencies

All Python dependencies are listed in the [`requirements.txt`](requirements.txt) file:

```bash
# Install all dependencies
pip install -r requirements.txt

# For development with specific versions
pip install -r requirements.txt --upgrade
```

### Main Dependencies:
- **ROS2 Python packages**: `rclpy`, `std-msgs`, `geometry-msgs`
- **Motor control**: `pyserial` for Sabertooth communication
- **Visualization**: `matplotlib`, `numpy` for real-time plotting
- **Image processing**: `opencv-python`, `Pillow` (optional)
- **Development**: `pytest`, `black`, `flake8`

### Running Tests

To run the test suite:
```bash
# Install test dependencies
pip install -r requirements.txt

# Run tests
pytest src/ -v

# Run with coverage
pytest src/ --cov=.
```

### Code Quality

Use the included tools for code quality:
```bash
# Format code with black
black src/

# Check code style with flake8
flake8 src/
```

## Collaboration
- **University of Milan**: Partner for accessibility validation.
- **Contributions**: Pull requests and issues are welcome!
- **License**: Apache 2.0 (see [LICENSE](LICENSE)).

---

## Contact
- **Maintainer**: Renaud JANET ([@RenaudFookIng](https://github.com/RenaudFookIng))
- **Support**: Open an [issue](https://github.com/RenaudFookIng/E-WheelChAIr/issues) for questions or suggestions.

