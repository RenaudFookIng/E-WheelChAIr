# EWheelChAIr ROS2 Workspace

**ROS2 Workspace for E-WheelChAIr Project**
*(Electric Wheelchair Control and Sensor Integration)*

---

## 📌 Description
This workspace contains all ROS2 packages for the **E-WheelChAIr** project, including:
- Motor control nodes.
- Sensor drivers (camera, ultrasonic).
- Custom message definitions.
- Launch files for system integration.

---

## 📂 Structure
```
EWheelChAIr_ws/
├── src/
│   ├── e_wheelchair_control/   # Motor control and safety logic
│   ├── e_wheelchair_sensors/   # Camera and ultrasonic sensor drivers
│   ├── e_wheelchair_msgs/      # Custom ROS2 messages
│   └── e_wheelchair_bringup/   # Launch files and system configuration
├── build/                      # Build directory (ignored by Git)
├── install/                    # Installation directory (ignored by Git)
├── log/                        # Log directory (ignored by Git)
└── README.md                   # This file
```

---

## 🛠 Prerequisites
- **ROS2 Humble** (or Foxy) installed.
- **colcon** build tool.
- **Python 3.8+** (for ROS2 scripts).
- **Dependencies**:
  ```bash
  sudo apt install ros-humble-realsense2-camera ros-humble-urg-node ros-humble-joint-state-publisher-gui
  ```

---

## 🚀 Setup and Build

### 1. Install Dependencies
```bash
sudo apt update
sudo apt install -y ros-humble-realsense2-camera ros-humble-urg-node
```

### 2. Build the Workspace
```bash
cd EWheelChAIr_ws
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### 3. Source the Workspace
```bash
source install/setup.bash
```

---

## 📦 Packages

| Package                  | Description                                                                 |
|--------------------------|-----------------------------------------------------------------------------|
| `e_wheelchair_control`   | Low-level motor control, safety checks, and joystick input handling.      |
| `e_wheelchair_sensors`   | Drivers for Intel Realsense camera and HC-SR04 ultrasonic sensors.         |
| `e_wheelchair_msgs`      | Custom ROS2 messages for wheelchair control and sensor data.               |
| `e_wheelchair_bringup`   | Launch files to start the full system (motors + sensors + visualization). |

---

## 🔧 Usage

### Launch the Full System
```bash
ros2 launch e_wheelchair_bringup system.launch.py
```

### Launch Only Motors
```bash
ros2 launch e_wheelchair_control motor_control.launch.py
```

### Launch Only Sensors
```bash
ros2 launch e_wheelchair_sensors sensors.launch.py
```

### Visualize in RViz
```bash
ros2 launch e_wheelchair_bringup view.launch.py
```

---

## 🔌 Topics and Services

### Published Topics
| Topic Name                | Type                          | Description                                  |
|---------------------------|-------------------------------|----------------------------------------------|
| `/wheelchair/cmd_vel`     | `geometry_msgs/Twist`         | Command velocity for motors.                |
| `/wheelchair/sensor_data` | `e_wheelchair_msgs/SensorData`| Fused sensor data (ultrasonic + camera).      |
| `/wheelchair/status`      | `e_wheelchair_msgs/Status`    | Wheelchair status (battery, errors, etc.).   |

### Subscribed Topics
| Topic Name                | Type                          | Description                                  |
|---------------------------|-------------------------------|----------------------------------------------|
| `/joy`                    | `sensor_msgs/Joy`             | Joystick input.                              |

### Services
| Service Name              | Type                          | Description                                  |
|---------------------------|-------------------------------|----------------------------------------------|
| `/wheelchair/enable`      | `std_srvs/SetBool`            | Enable/disable motor control.                |

---

## 📝 Configuration
- **Motor parameters**: Edit `e_wheelchair_control/config/motor.yaml`.
- **Sensor parameters**: Edit `e_wheelchair_sensors/config/sensors.yaml`.
- **Launch arguments**: See `e_wheelchair_bringup/launch/system.launch.py`.

---

## 🔍 Debugging
- **View topics**:
  ```bash
  ros2 topic list
  ros2 topic echo /wheelchair/status
  ```
- **View nodes**:
  ```bash
  ros2 node list
  ```
- **Log output**:
  ```bash
  ros2 launch e_wheelchair_bringup system.launch.py > wheelchair.log 2>&1
  ```

---

## 📄 License
This workspace is part of the **E-WheelChAIr** project, licensed under **Apache 2.0**.
See [../LICENSE](../LICENSE) for details.

