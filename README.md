# ROS 2 Gazebo Simulation – Elephant CoRobot-pi Manipulator

## Overview
This repository contains a **simulation-first ROS 2–Gazebo pipeline** for the *Elephant CoRobot-pi / myCobot* manipulator, developed **without official manufacturer URDF or Gazebo support**.

---

## What I Did

- Built a **physically consistent, deployable URDF** from scratch using limited RViz-only references
- Validated **kinematics, joint limits, and link structure** to ensure realistic motion
- Created a complete **ROS 2 + Gazebo manipulation simulation** 
- Integrated **`ros2_control`** with custom controller configurations for stable trajectory execution
---


## Requirements

- Ubuntu 24.04
- ROS 2 Jazzy
- Gazebo (Harmonic)
- `ros2_control`
- `ros2_controllers`

---

## Build Instructions

Create a workspace and clone the repository:

```bash
git clone https://github.com/mltejas88/Project_Mycobot_280pi_simulation.git
source /opt/ros/jazzy/setup.bash
cd ~/mycobot/workspace
rm -rf build install log
colcon build
source install/setup.bash


