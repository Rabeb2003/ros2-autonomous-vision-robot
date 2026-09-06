# ROS 2 Autonomous Robot: Vision & Navigation

> **An integrated simulation framework leveraging YOLOv8 perception and Nav2 for autonomous differential drive robots.**

## 📖 Research Context

In autonomous robotics, integrating robust visual perception with reliable navigation remains a significant challenge. This project serves as an experimental testbed for evaluating real-time perception algorithms (YOLOv8) in dynamic environments while maintaining stable autonomous navigation.

Developed during a mid-semester research internship, this repository demonstrates a complete ROS 2 Humble pipeline from sensor simulation in Gazebo to high-level cognitive tasks (person detection) and motion planning.

## ✨ Technical Contributions

- **Real-Time Perception Pipeline:** Integrated YOLOv8 via Python and `cv_bridge` to perform real-time person detection (CPU-optimized) using simulated RGB camera feeds.
- **Advanced Navigation & Path Planning:** Implemented custom A* and RRT algorithms alongside the standard Nav2 stack for autonomous waypoint navigation.
- **Robust Simulation Environment:** Designed custom Gazebo worlds populated with animated actors to evaluate the perception and navigation stacks under dynamic occlusion.

## 🛠️ Technology Stack

| Category | Technologies / Frameworks |
| :--- | :--- |
| **Middleware** | ROS 2 Humble |
| **Simulation** | Gazebo Classic (11.10), RViz2 |
| **Perception** | OpenCV, Ultralytics (YOLOv8), `cv_bridge` |
| **Navigation & Planning**| Nav2, Custom RRT & A*, TF2 |
| **Languages** | Python 3, XML (URDF/Xacro) |

## 🏗️ System Architecture

```text
[ Gazebo Simulation ]
      │ (RGB Camera)           (LiDAR / Odometry)
      ▼                             ▼
[ Perception Node ]          [ Navigation Stack ]
  - YOLOv8 Inference           - Nav2 / AMCL
  - Bounding Boxes             - Custom RRT / A*
      │                             │
      ▼                             ▼
[ Visualizer ]               [ Control Layer ]
  - rqt_image_view             - diff_drive_controller
  - RViz2                      
```

## ⚙️ Requirements & Installation

- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Python:** Python 3.10+, `numpy<2` (for `cv_bridge` compatibility), `ultralytics`, `opencv-python`

```bash
# 1. Install ROS 2 dependencies
sudo apt update && sudo apt install -y ros-humble-desktop ros-humble-gazebo-ros-pkgs ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-robot-localization ros-humble-xacro

# 2. Install Python dependencies
pip3 install "numpy<2" ultralytics opencv-python --user

# 3. Clone the repository
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/Rabeb2003/ros2-autonomous-vision-robot.git

# 4. Build the workspace
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 🚀 Build & Run (Demo)

This demo launches the robot in a dynamic Gazebo environment, starts the YOLOv8 person detection node, and visualizes the results in RViz2 and `rqt_image_view`.

```bash
source install/setup.bash

# Launch the full simulation with perception
ros2 launch diff_robot person_detection_demo.launch.py
```

## 📊 Validation & Results

- **Validation:** Qualitative validation has been achieved in Gazebo simulation. The perception node successfully identifies animated actors at 15+ FPS on CPU, while the navigation stack routes the robot safely through cluttered environments.
- **Limitations:** The current implementation relies on Gazebo Classic. Future work includes migrating to Gazebo Ignition/Harmonic and deploying the stack onto physical hardware.


---
**Author:** Rabeb Bouzaida — ENIM (Tunisia) — Electrical Engineering  
**GitHub:** [@Rabeb2003](https://github.com/Rabeb2003)
