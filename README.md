# Unitree B2 - Hierarchical Sensor Fusion and SLAM

This repository contains the source code and configuration files for the bachelor's thesis: **"Simultaneous Localization and Mapping for a Quadruped Robot"**. 

The project implements a hierarchical sensor fusion architecture for the Unitree B2 legged robot using ROS 2 Humble. It evaluates multiple levels of localization (from basic kinematics to 3D LiDAR odometry and global map-based corrections) to significantly reduce trajectory drift.

## Features

The architecture is divided into 4 localization levels (L1 - L4) to perform a detailed ablation study:
- **L1 (Base Odometry):** Internal Unitree SDK kinematics and IMU (Black-box).
- **L2 (Local Fusion):** 2D Kinematics fused with raw external IMU data using an Extended Kalman Filter (EKF).
- **L3 (LiDAR-Inertial Odometry):** Fusion of L2 with 3D LiDAR data (Velodyne VLP-16) using **KISS-ICP** / **FastLIO2**.
- **L4 (Global Localization):** L3 odometry corrected globally against a pre-built map using **AMCL** (Adaptive Monte Carlo Localization).

## Prerequisites

To run this project natively, you need the following environment:
- **OS:** Ubuntu 22.04 LTS
- **ROS 2:** Humble Hawksbill
- **Tools:** `colcon`, `rosdep`, `evo` (for trajectory evaluation)

## Installation & Build

Clone this repository and build the workspace. Docker is not required; the solution is designed to run natively on the host system.

```bash
# 1. Create a ROS 2 workspace
mkdir -p ~/b2_ws/src
cd ~/b2_ws/src

# 2. Clone this repository
git clone <YOUR_GITHUB_REPOSITORY_URL> b2_thesis

# 3. Install dependencies using rosdep
cd ~/b2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install
