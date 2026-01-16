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

**Note:** All ROS launch files are centralized in the `e_wheelchair_launch` package for easier management and management.

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
│       ├── servo_controller  # NOUVEAU: Contrôle des servos MZ996
│       │   ├── servo_controller
│       │   │   ├── __init__.py
│       │   │   └── servo_controller_node.py
│       │   ├── resource
│       │   │   └── config.yaml
│       │   ├── servo_controller.ino  # Sketch Arduino
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
|   │   │   ├── DetectedObject.msg
|   │   │   ├── DetectedObjectArray.msg
|   │   │   ├── EmergencyData.msg
|   │   │   ├── Joystick.msg
|   │   │   ├── ObstacleDetection.msg
|   │   │   ├── ServoCommand.msg
|   │   │   ├── UltrasonicArray.msg
|   │   │   └── WyesIntent.msg
|   │   └── package.xml
|   ├── e_wheelchair_launch
|   │   ├── e_wheelchair_launch
|   |   │   ├── __init__.py
|   |   │   └── launch
|   |   │       ├── ewheelchair_all.launch.py      # Complete system launch
|   |   │       ├── servo_controller.launch.py     # Servo control launch
|   |   │       └── wide_camera.launch.py         # Camera processing launch
|   │   ├── package.xml
|   │   └── setup.py
|   ├── wide_processing 
|   │   ├── resource
|   │   ├── test
|   │   ├── wide_processing
|   │   │   ├── __init__.py
|   │   │   └── wide_processing_node.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── depth_processing
|   │   ├── resource
|   │   ├── test
|   │   ├── depth_processing
|   │   │   ├── __init__.py
|   │   │   └── depth_processing_node.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── master_node  # MODIFIÉ: Gestion des servos au lieu des moteurs
|   │   ├── config
|   │   ├── master_node
|   │   │   ├── __init__.py
|   │   │   └── master_node.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── arduino_data_receiver
|   │   ├── arduino_data_receiver
|   │   │   ├── __init__.py
|   │   │   └── arduino_data_receiver_node.py
|   │   ├── resource
|   │   │   └── arduino_data_receiver
|   │   ├── test
|   │   │   ├── test_copyright.py
|   │   │   ├── test_flake8.py
|   │   │   └── test_pep257.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   ├── servo_controller
|   │   ├── config
|   │   │   └── servo_config.yaml
|   │   ├── resource
|   │   │   └── servo_controller
|   │   ├── servo_controller
|   │   │   ├── __init__.py
|   │   │   └── servo_controller_node.py
|   │   ├── package.xml
|   │   ├── setup.cfg
|   │   └── setup.py
|   └── wyes_teleop
|       ├── setup.cfg
|       ├── package.xml
|       ├── setup.py
|       ├── resource
|       │   └── wyes_teleop
|       ├── test
|       │   ├── test_copyright.py
|       │   ├── test_flake8.py
|       │   └── test_pep257.py
|       └── wyes_teleop
|           ├── __init__.py
|           └── wyes_teleop_node.py
|
├── LICENSE
└── README.md
```

---

## Prerequisites

### Hardware
- Electric wheelchair with servo-controlled joystick (using Miuzei MG996 servos).
- Sensors: Intel Realsense (depth camera), HC-SR04 (ultrasonic), Arduino (I/O interface).
- Joystick or alternative control input device.

### Software
- ROS2 Humble (or Foxy).
- colcon (build tool for ROS2).
- FreeCAD 0.20+ (for 3D modeling).
- Git LFS (for versioning large 3D files).

---

## Installation and Setup

### 1. Clone the Repository
```bash
git clone https://github.com/RenaudFookIng/E-WheelChAIr.git
cd E-WheelChAIr_ws
```

### 2. Build the ROS Workspace
```bash
cd EWheelChAIr_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the System
```bash
ros2 launch e_wheelchair_launch ewheelchair_all.launch.py
```

---

## ROS Launch Files Centralization

All ROS launch files have been centralized in the `e_wheelchair_launch` package to provide:

- **Simplified Management**: Single location for all launch configurations
- **Easier Maintenance**: No duplication across packages
- **Consistent Structure**: Uniform approach to system launching
- **Better Organization**: Clear separation between nodes and launch configurations

### Available Launch Files

| Launch File | Description | Components Launched |
|-------------|-------------|---------------------|
| `ewheelchair_all.launch.py` | Complete system with all sensors and processing | 6 nodes |
| `servo_controller.launch.py` | Direct servo control interface | 1 node |
| `wide_camera.launch.py` | Wide-angle camera processing | 1 node |

## ROS Packages

| Package               | Description                                  |
|-----------------------|----------------------------------------------|
| `master_node`          | Main control node for servo-controlled joystick |
| `servo_controller`     | Controls Miuzei MG996 servos via Arduino     |
| `arduino_data_receiver` | Receives data from Arduino sensors       |
| `depth_processing`     | Processes depth camera data                  |
| `wide_processing`      | Processes wide-angle camera data             |
| `wyes_teleop`          | Keyboard-based teleoperation interface        |
| `custom_msgs`          | Custom ROS message definitions               |
| `e_wheelchair_launch`  | Launch files for the complete system         |

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

## Recent Changes

### Version 0.2.0 (Current)
- **Major Architecture Change**: Replaced Sabertooth motor controller with Miuzei MG996 servo-based joystick control
- **New Package**: `servo_controller` for Arduino-based servo management
- **Updated**: `master_node` now publishes joystick commands instead of motor commands
- **Updated**: `visualization` package removed motor speed plotting
- **Removed**: `sabertooth_controller` and `motor_speed_calculator` packages
- **Improved**: Safety features with neutral position on emergency stop

### Version 0.1.0
- Initial release with Sabertooth motor controller
- Basic sensor integration (ultrasonic, camera)
- ROS2 Humble compatibility

## Collaboration
- **University of Milan**: Partner for accessibility validation.
- **Contributions**: Pull requests and issues are welcome!
- **License**: Apache 2.0 (see [LICENSE](LICENSE)).

---

## Contact
- **Maintainer**: Renaud JANET ([@RenaudFookIng](https://github.com/RenaudFookIng))
- **Support**: Open an [issue](https://github.com/RenaudFookIng/E-WheelChAIr/issues) for questions or suggestions.

