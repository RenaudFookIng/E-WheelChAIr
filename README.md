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
|   ├── custom_msgs
|   │   ├── CMakeLists.txt
|   │   ├── msg
|   │   │   ├── EmergencyData.msg
|   │   │   ├── Joystick.msg
|   │   │   ├── ObstacleDetection.msg
|   │   │   └── UltrasonicArray.msg
|   │   └── package.xml
|   ├── e_wheelchair_launch
|   │   ├── e_wheelchair_launch
|   |   │   ├── __init__.py
|   |   │   └── launch
|   |   │       └── ewheelchair_all.launch.py
|   │   ├── package.xml
|   │   └── setup.py
|   ├── image_processing
|   │   ├── config
|   │   │   ├── camera_info.yaml
|   │   │   ├── depth_camera_params.yaml
|   │   │   └── wide_camera_params.yaml
|   │   ├── launch
|   │   │   ├── depth_camera.launch.py
|   │   │   └── wide_cameras.launch.py
|   │   └── src
|   │       ├── depth_camera_driver
|   │       ├── depth_processing
|   │       ├── wide_camera_driver
|   │       └── wide_processing
|   ├── lidar
|   │   ├── launch
|   │   │   └── lidar.launch.py
|   │   ├── lidar
|   │   │   └── __init__.py
|   │   ├── package.xml
|   │   ├── resource
|   │   ├── setup.py
|   │   └── test
|   ├── master_node
|   │   ├── CMakeLists.txt
|   │   ├── master_node
|   │   │   ├── __init__.py
|   │   │   └── master_node.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── motor_speed_calculator
|   │   ├── CMakeLists.txt
|   │   ├── motor_speed_calculator
|   │   │   ├── __init__.py
|   │   │   └── motor_speed_calculator.py
|   │   ├── package.xml
|   │   └── setup.py
|   ├── sabertooth_controller
|   │   ├── CMakeLists.txt
|   │   ├── include
|   │   │   └── sabertooth_controller
|   │   ├── package.xml
|   │   └── src
|   │       └── sabertooth_controller.cpp
|   └── visualization
|       ├── CMakeLists.txt
|       ├── package.xml
|       ├── setup.py
|       └── visualization
|           ├── __init__.py
|           └── real_time_plot.py|
|
├── LICENSE
└── README.md
```

---

## Prerequisites

### Hardware
- Electric wheelchair with Sabertooth 2x32A motor driver (or equivalent).
- Sensors: Intel Realsense (depth camera), HC-SR04 (ultrasonic), Arduino (I/O interface).
- Joystick or alternative control input device.

### Software

#### For Raspberry Pi 3 (Python-only version):
- **ROS2 Humble** (recommended for Raspberry Pi 3)
- **Python 3.8+**
- **Required packages:**
  ```bash
  sudo apt install python3-colcon-common-extensions \
                   ros-humble-rclpy \
                   ros-humble-geometry-msgs \
                   python3-pyserial
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

### 2. Build the ROS Workspace

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

### Original Packages (C++ - Problematic for Raspberry Pi 3)
| Package               | Description                                  |
|-----------------------|----------------------------------------------|
| `sabertooth_controller` | C++ motor controller (replaced by Python version) |
| `custom_msgs`          | C++ custom messages (replaced by Python interface) |

### New Python-Only Packages (Raspberry Pi 3 Compatible)
| Package                     | Description                                  |
|-----------------------------|----------------------------------------------|
| `sabertooth_controller_py`  | Python motor controller using pyserial       |
| `custom_msgs_py`            | Python interface for custom messages         |
| `master_node`               | Central control node (updated for Python)    |
| `motor_speed_calculator`    | Motor speed calculation                     |
| `e_wheelchair_launch`       | Launch files including Python-only version   |

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
This project now includes a **Python-only version** specifically designed for Raspberry Pi 3 compatibility:

- **✅ No C++ dependencies** - All motor control and message handling in Python
- **✅ Reduced memory footprint** - Python is more memory-efficient than C++ on Raspberry Pi 3
- **✅ Easier debugging** - Python stack traces and logging
- **✅ Faster iteration** - No compilation needed for Python changes

### Performance Considerations
- The Python version uses `pyserial` instead of `serial_driver` for motor control
- Message processing is handled by Python classes instead of ROS2 C++ messages
- All safety features (emergency stop, obstacle detection) are preserved

### Known Limitations
- Some advanced features (RealSense camera, LiDAR) may require additional optimization
- For best performance, use ROS2 Humble with its Python optimizations

## Collaboration
- **University of Milan**: Partner for accessibility validation.
- **Contributions**: Pull requests and issues are welcome!
- **License**: Apache 2.0 (see [LICENSE](LICENSE)).

---

## Contact
- **Maintainer**: Renaud JANET ([@RenaudFookIng](https://github.com/RenaudFookIng))
- **Support**: Open an [issue](https://github.com/RenaudFookIng/E-WheelChAIr/issues) for questions or suggestions.

