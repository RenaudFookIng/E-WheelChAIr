# E-WheelChAIr
**Smart Embedded System for Electric Wheelchairs**

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![ROS2](https://img.shields.io/badge/ROS2-Humble-22314E?logo=ros)](https://docs.ros.org/en/humble/)

---

## Description
E-WheelChAIr is an open-source project designed to enhance autonomy and accessibility for electric wheelchairs. It integrates:
- A **ROS2-based architecture** for real-time control and sensor fusion.
- **3D-printed mechanical parts** (sensor mounts, joystick adapters).
- **Modular design** for easy customization and collaboration with partners like the University of Milan.

---

## Project Structure
```
E-WheelChAIr/
|
├── 3d_models/            # 3D designs (FreeCAD/SolidWorks)
│   ├── camera_ultrasound_module/ # Camera and ultrasonic sensor housing
│   ├── joystick_mount/   # Custom joystick mount
│   └── README.md         # 3D models documentation
|
├── arduino_sensors/      # codes arduino
│   ├── joystick/
│   │   ├── joystick/
│   │   |   ├── __init__.py
│   │   |   └── wide_camera_node.py
│   │   ├── test/
│   │   |   └── test_joystick_node.py
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── ultrasonic/
│       ├── ultrasonic/
│       |   ├── __init__.py
│       |   └── ultrasonic_node.py
│       ├── resource/
│       |   └── config.yaml
│       ├── test/
│       |   └── test_ultrasonic_node.py
│       ├── package.xml
│       └── setup.py
|
├── image_processing/
|   ├── config/
|   │   ├── depth_camera_params.yaml
|   │   └── wide_camera_params.yaml
|   ├── launch/
|   │   ├── depth_camera.launch.py
|   │   └── wide_cameras.launch.py
|   ├── src/
|   │   ├── depth_camera_driver/    # Pour la depth camera (ex: RealSense)
|   │   │   ├── depth_camera_driver
|   │   │   |   ├── __init__.py
|   │   │   |   └── depth_camera_node.py
|   │   │   ├── package.xml
|   │   │   └── setup.py
|   │   │
|   │   ├── wide_camera_driver/     # Pour les caméras Fit0892
|   │   │   ├── wide_camera_driver
|   │   │   |   ├── __init__.py
|   │   │   |   └── wide_camera_node.py
|   │   │   ├── package.xml
|   │   │   └── setup.py
|   │   │
|   │   ├── depth_processing/       # Traitement pour la depth camera
|   │   │   ├── depth_processing
|   │   │   |   ├── __init__.py
|   │   │   |   └── depth_processing_node.py
|   │   │   ├── package.xml
|   │   │   └── setup.py
|   │   │
|   │   └── wide_processing/        # Traitement pour les caméras grand angle (YOLO + Depth Anything)
|   │       ├── resource/           # Modèles YOLO/Depth Anything
|   │       ├── wide_processing
|   │       |   ├── __init__.py
|   │       |   └── wide_processing_node.py
|   │       ├── package.xml
|   │       └── setup.py
|   │
|   ├── package.xml
|   ├── setup.cfg
|   └── setup.py
|
├── raspberry_pi_master/
│   ├── config/
│   ├── launch/
│   └── src/
|       ├── custom_msgs/
|       │   ├── msg/
|       |   |   ├── EmergencyData.msg
|       |   |   ├── Joystick.msg
|       |   |   ├── UltrasonicArray.msg
|       |   |   └── ObstacleDetection.msg
|       │   ├── CMakeLists.txt
|       │   └── package.xml
|       │
|       ├── lidar
|       │   ├── launch/
|       |   |   └── lidar.launch.py
|       │   ├── lidar/
|       |   │   └── __init__.py
|       │   ├── resource/
|       |   │   └── config.yaml
|       │   ├── test/
|       │   ├──setup.py
|       │   └──package.xml
|       │
|       ├── master_node
|       │   ├── master_node/
|       |   │   ├── __init__.py
|       |   │   └── master_node.py
|       │   ├── setup.py
|       │   ├── setup.cfg
|       │   ├── CMakeLists.txt
|       │   └── package.xml
|       │
│       ├── motor_speed_calculator/
|       │   ├── motor_speed_calculator/
│       │   │   ├── __init__.py
|       │   |   └── motor_speed_calculator.py
|       │   ├── package.xml
|       │   ├── setup.py
|       |   └── CMakeLists.txt
|       |
│       ├── visualization/
|       │   ├── visualization/
│       │   │   ├── __init__.py
|       │   |   └── real_time_plot_node.py
|       │   ├── package.xml
|       │   ├── setup.py
|       |   └── CMakeLists.txt
|       |
|       └── sabertooth_controller
|           ├── include/sabertooth_controller/
│           │   └── sabertooth_controller.hh
|           ├── src/
│           │   └── sabertooth_controller.cc
|           ├── CMakeLists.txt
|           └── package.xml
|
├── src/
├── utils/
├── docs/                 # Documentation 
├── .gitignore
├── LICENSE               # Apache 2.0 License
└── README.md             # This file
```

---

## Prerequisites

### Hardware
- Electric wheelchair with Sabertooth 2x32A motor driver (or equivalent).
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
cd E-WheelChAIr
```

### 2. Build the ROS Workspace
```bash
cd EWheelChAIr_ws
colcon build --symlink-install
source install/setup.bash
```

### 3. Launch the System
```bash
ros2 launch e_wheelchair_bringup system.launch.py
```
*(Replace `e_wheelchair_bringup` with your actual launch package name.)*

---

## ROS Packages

| Package               | Description                                  |
|-----------------------|----------------------------------------------|
| `e_wheelchair_control` | Low-level motor control and safety logic     |
| `e_wheelchair_sensors` | Driver for camera and ultrasonic sensor fusion |
| `e_wheelchair_msgs`    | Custom ROS message definitions               |

---

## 3D Models
- **Location**: [`3d_models/`](3d_models/)
- **Formats**: STL and STEP files for 3D printing and simulation.
- **Tools**: Designed in OnShape.
- **Note**: Use `git lfs pull` after cloning to retrieve 3D files.

---

## Collaboration
- **University of Milan**: Partner for accessibility validation.
- **Contributions**: Pull requests and issues are welcome!
- **License**: Apache 2.0 (see [LICENSE](LICENSE)).

---

## Contact
- **Maintainer**: Renaud JANET ([@RenaudFookIng](https://github.com/RenaudFookIng))
- **Support**: Open an [issue](https://github.com/RenaudFookIng/E-WheelChAIr/issues) for questions or suggestions.

