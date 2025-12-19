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
- RViz Visualization
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

