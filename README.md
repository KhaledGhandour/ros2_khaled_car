# ros2_khaled_car

ROS 2 Jazzy skid-steering four-wheel mobile robot simulation using Gazebo Harmonic.

This repository contains a complete simulation and control setup for a **four-wheel skid-steering mobile robot**
built with **ROS 2 Jazzy** and **Gazebo Harmonic (gz-sim)**.
The robot supports multiple onboard sensors including **LiDAR, camera, ultrasonic sensors, and IMU**,
and can be controlled using velocity commands (`/cmd_vel`) via keyboard teleoperation or autonomous nodes.

---

## 📌 Table of Contents
- Overview
- Features
- Robot Architecture
- Sensors
- Repository Structure
- Installation
- Build
- Running the Simulation
- Teleoperation
- Topics & Interfaces
- Extensions
- License

---

## 🔍 Overview

This project is designed as a learning and development platform for:
- Skid-steering mobile robots
- ROS 2 control architecture
- Gazebo Harmonic simulation
- Sensor integration and visualization
- Future autonomous navigation experiments

The robot uses **ros2_control** with a skid-steer configuration to drive four wheels in simulation.

---

## 🚀 Features

- ✅ ROS 2 Jazzy compatible
- ✅ Gazebo Harmonic (gz-sim) simulation
- ✅ Four-wheel skid-steering drive system
- ✅ ros2_control integration
- ✅ Velocity control using `/cmd_vel`
- ✅ LiDAR sensor
- ✅ Camera sensor
- ✅ Ultrasonic (range) sensors
- ✅ IMU sensor
- ✅ RViz2 visualization (TF, robot model, sensor data)
- ✅ Modular URDF/Xacro design

---

## 🤖 Robot Architecture

- **Drive type:** Skid steering (4 wheels)
- **Controller:** ros2_control skid-steer / diff-based controller
- **Base frame:** `base_link`
- **Wheel joints:** Velocity-controlled joints
- **Simulation:** Gazebo Harmonic with gz_ros2_control plugin

---

## 📡 Sensors

The robot includes the following simulated sensors:

### LiDAR
- Publishes `sensor_msgs/LaserScan`
- Used for mapping, obstacle detection, and navigation

### Camera
- Publishes `sensor_msgs/Image` and `sensor_msgs/CameraInfo`
- Useful for perception and vision-based algorithms

### Ultrasonic Sensors
- Publishes `sensor_msgs/Range`
- Used for short-range obstacle detection

### IMU
- Publishes `sensor_msgs/Imu`
- Provides orientation, angular velocity, and acceleration data

---

## 📁 Repository Structure

*(Structure may slightly vary depending on updates)*

---

## 🛠 Installation

### Prerequisites
- Ubuntu 24.04 (recommended)
- ROS 2 Jazzy
- Gazebo Harmonic

### Clone the Repository
```bash
git clone https://github.com/KhaledGhandour/ros2_khaled_car.git
cd ros2_khaled_car 
```


 ## Install Dependencies
```
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y
```
---
## 🏗 Build the Workspace
``` colcon build
source install/setup.bash
```
---
## ▶️ Running the Simulation

Launch Gazebo with the robot:
```
ros2 launch k_FWD_robot_sim fws_robot_spawn.launch.py 
```
This will:

- Start Gazebo Harmonic

- Spawn the skid-steering robot

- Load ros2_control controllers

- Start sensor plugins
---
## 🎮 Teleoperation

Control the robot using keyboard teleoperation:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


Key controls:

-  W / X → Forward / backward

-  A / D → Rotate left / right

- S → Stop
---

## 🔁 Topics & Interfaces

Common ROS topics used in this project
- /cmd_vel
- /odom
- /joint_states
- /scan
- /camera/image_raw
- /camera/camera_info
- /imu/data
- /range
- /tf
- /tf_static
---
## 🧠 Extensions & Future Work

This project can be extended to include:

- Navigation2 (Nav2) stack

- SLAM (SLAM Toolbox)

- EKF localization (robot_localization)

- Autonomous path planning

- Sensor fusion

- Real robot hardware integration

## 📜 License

This project is released under the MIT License.

## ⭐ Acknowledgments

- ROS 2 community

- Gazebo / gz-sim developers

- Open-source robotics contributors
