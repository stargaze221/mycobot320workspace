
---

````markdown
# ⚙️ Hardware Setup Guide — MyCobotProject

This document describes how to physically connect and configure all hardware components for **MyCobotProject**.  
The setup integrates **MyCobot 320**, **stereo/depth cameras (RealSense D455 and ZED 2i)**, and an **NVIDIA AGX Orin** running **ROS 2 Humble** inside Docker.

---

## 🦾 1. MyCobot 320 Arm

### Overview
The **Elephant Robotics MyCobot 320** is a 6-DOF collaborative robot with a built-in CH340 USB-to-Serial interface for communication.

| Spec | Description |
|------|--------------|
| Controller | M5 or Pi base depending on version |
| Degrees of Freedom | 6 |
| Communication | USB (Serial @ 115200 baud) |
| Power | 12 V DC (≥ 5 A) |
| End-Effector | Optional servo gripper (PWM-controlled) |

### Connection Steps
1. Connect the **robot’s main USB cable** to the AGX Orin (preferably a USB 3.0 port).  
2. Power on the robot using the external 12 V adapter.  
3. Verify detection:
   ```bash
   lsusb | grep -i qinheng
````

Expected: `ID 1a86:55d4 QinHeng Electronics USB Single Serial`
4. Apply the custom [udev rule](UdevRules.md) so the robot always appears as `/dev/mycobot320`.
5. Test the connection:

```bash
python3 -m pymycobot.test --port /dev/mycobot320
```

---

## 🎥 2. Cameras

### Intel RealSense D455 (Lower Mount)

Used for close-range depth and RGB sensing.

| Spec         | Value                                                         |
| ------------ | ------------------------------------------------------------- |
| Vendor ID    | `8086`                                                        |
| Product ID   | `0b5c`                                                        |
| Connection   | USB 3.1 Type-C                                                |
| ROS 2 Driver | `isaac_ros_realsense` or `zed-ros2-wrapper` equivalent topics |

**Udev Rule (optional)**
Add to `/etc/udev/rules.d/99-realsense.rules`:

```bash
SUBSYSTEM=="usb", ATTR{idVendor}=="8086", ATTR{idProduct}=="0b5c", MODE="0666", GROUP="plugdev"
```

Mount the D455 **below or beside the end-effector** for workspace mapping and calibration.

---

### ZED 2i (Upper Mount)

Provides high-resolution stereo and inertial data for NVBlox and Visual SLAM.

| Spec         | Value                                            |
| ------------ | ------------------------------------------------ |
| Vendor ID    | `2b03`                                           |
| Product ID   | `f681`                                           |
| Power        | USB 3.0 with external 12 V optional              |
| ROS 2 Driver | `zed-ros2-wrapper (v4.2.2)`                      |
| SDK          | ZED SDK 4.2.2 + CUDA 12.6 (JetPack 6 / L4T 36.4) |

Mount the ZED 2i **higher and facing the workspace** to capture a wider 3-D scene.

---

## 🧠 3. Compute Platform — NVIDIA AGX Orin

| Component         | Specification                                |
| ----------------- | -------------------------------------------- |
| OS                | JetPack 6 (L4T 36.4, Ubuntu 22.04)           |
| GPU               | Ampere architecture (2048 CUDA cores)        |
| RAM               | 32 GB                                        |
| Storage           | NVMe SSD (recommended ≥ 256 GB)              |
| ROS 2             | Humble Hawksbill                             |
| Container Runtime | NVIDIA Container Toolkit (CUDA 12.6 enabled) |

### Recommended Setup

1. Install **NoMachine** or SSH for headless remote access.
   You can use the **dummy monitor (Xorg)** method described in `DummyMonitorSetup.md`.
2. Connect AGX Orin to the same LAN as your laptop for remote development.
3. Use a powered USB hub if multiple cameras and the MyCobot are attached.

---

## 🔌 4. Power and Cable Management

| Device         | Power Source             | Notes                                                  |
| -------------- | ------------------------ | ------------------------------------------------------ |
| MyCobot 320    | 12 V DC (5–6 A)          | Stable power required for servo motion                 |
| ZED 2i         | USB 3.0 / 12 V auxiliary | Avoid undervoltage by using external adapter if needed |
| RealSense D455 | USB 3.0                  | Bus-powered                                            |
| AGX Orin       | 19 V / 10 A supply       | Use official power adapter                             |

Use **cable sleeves and strain reliefs** to prevent tension on USB and power connectors, especially around robot joints.

---

## 🧩 5. ROS 2 Node Overview

| Component         | ROS 2 Node                           | Description                                          |
| ----------------- | ------------------------------------ | ---------------------------------------------------- |
| MyCobot Interface | `mycobot_interface_node.py`          | Communicates with MyCobot via `/dev/mycobot320`      |
| Camera Drivers    | `realsense_node`, `zed_wrapper_node` | Publish image, depth, IMU topics                     |
| NVBlox            | `nvblox_node`                        | Builds 3-D TSDF / occupancy map                      |
| State Estimation  | `belief_planner_node`                | Fuses sensor data for uncertainty-aware pose         |
| Control           | `controller_node`                    | Executes Crocoddyl trajectories via Pinocchio models |

---

## 🧪 6. Verification Checklist

| Task               | Command                             | Expected Result                   |                              |
| ------------------ | ----------------------------------- | --------------------------------- | ---------------------------- |
| Verify robot port  | `ls -l /dev/mycobot320`             | Symlink exists                    |                              |
| Ping cameras       | `ros2 topic list                    | grep image`                       | RGB and depth topics visible |
| Test motion        | Run `mycobot_diagnostic.py`         | Angles update, no errors          |                              |
| Test mapping       | Launch `nvblox_realsense.launch.py` | Depth map and TSDF update in RViz |                              |
| Test visualization | Run `meshcat_visualizer.py`         | Joint model shown in browser      |                              |

---

## 🧰 7. Troubleshooting

* **USB permission errors:** confirm [udev rules](UdevRules.md) and user in `dialout` group.
* **ZED not detected:** use powered USB port and verify `udevadm info`.
* **Robot not moving:** check power supply and `mc.is_power_on()` state.
* **Docker cannot access USB:** launch container with `--privileged --device /dev/mycobot320`.

---

✅ **Result:**
All hardware components integrate seamlessly within the ROS 2 ecosystem on AGX Orin, providing synchronized sensing, control, and visualization for adaptive robot experimentation.

```

---

Would you like me to make a matching `Dependency.md` next — detailing the JetPack 6, ROS 2 Humble, and required Python / system packages (Pinocchio, Crocoddyl, MeshCat, PyMyCobot, Isaac ROS, etc.)?
```
