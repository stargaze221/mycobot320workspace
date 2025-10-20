

---

````markdown
# 📦 Dependency and Environment Setup — MyCobotProject

This document describes all software dependencies and installation steps required to reproduce the **MyCobotProject** environment on **NVIDIA AGX Orin** (JetPack 6 / Ubuntu 22.04 / ROS 2 Humble).

---

## 🧠 1. System Overview

| Component | Version / Notes |
|------------|----------------|
| **OS** | JetPack 6 (L4T 36.4 – Ubuntu 22.04) |
| **ROS 2** | Humble Hawksbill |
| **CUDA / cuDNN** | CUDA 12.6 included with JetPack |
| **Python** | 3.10 (default) |
| **C++ Standard** | C++17 |
| **Build System** | `colcon` with `ament_cmake` |

---

## 🧰 2. Core Dependencies

Install ROS 2 and essential developer tools:

```bash
sudo apt update && sudo apt install -y \
  build-essential cmake git python3-colcon-common-extensions \
  python3-pip python3-rosdep python3-vcstool \
  ros-humble-desktop ros-humble-rviz2 ros-humble-tf-transformations
````

Initialize `rosdep`:

```bash
sudo rosdep init
rosdep update
```

---

## 🐳 3. NVIDIA Container / Docker Environment

We recommend running inside a **Docker workspace** for reproducibility.

Install dependencies:

```bash
sudo apt install -y nvidia-container-toolkit docker.io
sudo systemctl enable --now docker
```

Verify GPU access:

```bash
sudo docker run --rm --runtime nvidia --gpus all nvidia/cuda:12.6.0-base nvidia-smi
```

Clone and build your workspace:

```bash
git clone --recurse-submodules https://github.com/stargaze221/isaac_ros-dev.git
cd isaac_ros-dev/src/isaac_ros_common
./scripts/run_dev.sh -d ${ISAAC_ROS_WS}
```

---

## 🔷 4. Isaac ROS and NVBlox

Used for visual SLAM and 3-D mapping.

| Package                 | Function                                       |
| ----------------------- | ---------------------------------------------- |
| `isaac_ros_nvblox`      | GPU TSDF / occupancy mapping                   |
| `isaac_ros_visual_slam` | VIO-based pose estimation                      |
| `isaac_ros_common`      | Base container utilities and environment setup |

To build inside Docker:

```bash
cd ${ISAAC_ROS_WS}
rosdep install --from-paths src -y --ignore-src
colcon build --symlink-install
```

---

## 🎥 5. Camera SDKs

### Intel RealSense D455

Install the official SDK:

```bash
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6B0FC61
sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main"
sudo apt update
sudo apt install -y librealsense2-utils librealsense2-dev
```

### ZED 2i

Download from [Stereolabs](https://www.stereolabs.com/developers/release/).
Use **ZED SDK 4.2.2** (JetPack 6 compatible) and verify with:

```bash
ZED_Diagnostic
```

ROS 2 wrapper (branch `humble-v4.2.5`):

```bash
git clone -b humble-v4.2.5 https://github.com/stargaze221/zed-ros2-wrapper.git
```

---

## 🦾 6. Robot Control – MyCobot 320

### Python API

Install the Elephant Robotics `pymycobot` library:

```bash
pip install pymycobot==3.4.3
```

### Serial Access

Ensure udev rule is configured as described in [UdevRules.md](UdevRules.md).
Confirm communication:

```bash
python3 -m pymycobot.test --port /dev/mycobot320
```

---

## 📐 7. Dynamics & Optimization Libraries

| Library       | Purpose                          |
| ------------- | -------------------------------- |
| **Pinocchio** | Rigid-body dynamics & kinematics |
| **Crocoddyl** | Optimal control solver (DDP)     |
| **MeshCat**   | Web-based 3-D visualization      |

Install via `pip` (Jetson-compatible):

```bash
pip install pin==2.9.2 crocoddyl==2.0.0 meshcat==0.3.2
```

If building from source:

```bash
sudo apt install -y libeigen3-dev libboost-all-dev
```

---

## 🧩 8. Supporting Packages

```bash
pip install numpy scipy matplotlib transforms3d pyserial tqdm scikit-optimize
```

Optional (for Braille / tactile interface):

```bash
pip install opencv-python pillow pyserial hdlc
```

---

## 🧠 9. Recommended Workspace Structure

```
~/mycobotproject/
│
├── isaac_ros-dev/               # NVIDIA perception/mapping stack
├── mycobot_interface/            # Robot driver and diagnostics
├── trajectory_optimizer/         # Pinocchio + Crocoddyl optimization
├── belief_planner/               # Particle filter / POMDP estimation
├── braille_interface/             # NVBlox → tactile map
└── docs/
    ├── Dependency.md
    ├── UdevRules.md
    ├── HardwareSetup.md
    └── SoftwareArchitecture.md
```

---

## 🧪 10. Quick Verification

```bash
ros2 run mycobot_interface mycobot_diagnostic.py
ros2 run zed_wrapper zed_wrapper
ros2 run nvblox_ros nvblox_node
ros2 run meshcat_visualizer visualize_robot.py
```

All nodes should start without missing-package errors.

---

✅ **Result:**
Your environment now supports the complete perception-to-control pipeline for **MyCobot 320**, including GPU-accelerated mapping, dynamics optimization, and tactile output for human-robot interaction.

```

---

Would you like me to make a **`Troubleshooting.md`** next (covering common issues like Docker USB access, missing CUDA libs, or RealSense/ZED detection failures)?
```
