# Unitree B2 - Hierarchical Sensor Fusion and SLAM

This repository contains the source code and configuration files for the bachelor's thesis: **"Simultaneous Localization and Mapping for a Quadruped Robot"**. 

The project implements a hierarchical sensor fusion architecture for the Unitree B2 legged robot using ROS 2 Humble. It evaluates multiple levels of localization (from basic kinematics to 3D LiDAR odometry and global map-based corrections) to significantly reduce trajectory drift.

## Features

The architecture is divided into 4 localization levels (L1 - L4) to perform a detailed фablation study, alongside a dedicated mapping module:
- **L1 (Base Odometry):** Internal Unitree SDK kinematics and IMU (Black-box).
- **L2 (Local Fusion):** 2D Kinematics fused with raw external IMU data using an Extended Kalman Filter (EKF).
- **L3 (LiDAR-Inertial Odometry):** Fusion of L2 with 3D LiDAR data (Velodyne VLP-16) using **KISS-ICP**.
- **L4 (Global Localization):** L3 odometry corrected globally against a pre-built map using **AMCL** (Adaptive Monte Carlo Localization).
- **3D Mapping:** Creation of high-quality 3D point cloud maps of the environment using **FastLIO2**.
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
git clone https://github.com/George-Chukhir/bachelor-thesis.git b2_thesis

# 3. Install dependencies using rosdep
cd ~/b2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build the workspace
colcon build --symlink-install

## Usage (Replication Manual)

After building, source your workspace to use the launch files.

```bash
source ~/b2_ws/install/setup.bash
```

### Running the Fusion (Example for L3)

**Terminal:** Launch the localization stack.

```bash
ros2 launch b2_thesis_fusion l3_fusion.launch.py 
```

## Evaluation Setup (evo)

To analyze trajectories using the `evo` tools, set up the specific virtual environment inside the `separate_trajectories` directory:

1. **Navigate to the target directory:**
   ```bash
   cd separate_trajectories
   ```

2. **Create the virtual environment named `evo_env`:**
   ```bash
   python3 -m venv evo_env
   ```
   
3. **Activate the environment:**
   ```bash
   source evo_env/bin/activate
   ```

4. **Install the required dependencies:**
   *Note: Ensure you point to the correct requirements file located in the project root.*
   ```bash
   pip install --upgrade pip
   pip install -r ../evo_requirements.txt


### Trajectory Evaluation
Once the rosbag finishes playing, you can quantitatively analyze the estimated trajectory against the Ground Truth (MoCap) using the `evo` package:

```bash
evo_ape tum mocap_gt.tum l3_traj.tum -p
```

## About the Author
- **Author:** Heorhii Chukhir
- **Supervisor:** Ing. Martin Lučan
- **Institution:** Slovak University of Technology in Bratislava (STU FEI), Institute of Robotics and Cybernetics.
- **Industry Partner:** Panza Robotics s.r.o.
