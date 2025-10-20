
---

```markdown
# 🧠 Software Architecture — MyCobotProject

This document describes the **software stack** that powers the MyCobotProject system — from low-level communication and state estimation to planning, control, and visualization.  
The system is designed to demonstrate how **computation, sensing, and learning** can overcome the mechanical limitations of low-cost robots.

---

## 🏗️ 1. System Overview

The architecture integrates perception, estimation, and control modules within a unified **ROS 2 Humble** framework running on **NVIDIA AGX Orin**.

```

Perception → State Estimation → Planning → Control → Visualization / HRI

```

| Layer | Main Packages / Nodes | Purpose |
|:------|:----------------------|:---------|
| **Perception** | `zed_wrapper_node`, `realsense_node`, `nvblox_node` | Capture RGB-D data, reconstruct environment, generate occupancy map |
| **State Estimation** | `belief_planner_node`, `particle_filter_node` | Fuse sensors, estimate robot and environment state with uncertainty |
| **Planning / Optimization** | `trajectory_optimizer_node`, `belief_planner_node` | Plan motion using Crocoddyl + Pinocchio, optionally uncertainty-aware |
| **Control** | `controller_node`, `mycobot_interface_node` | Execute optimized trajectory on physical robot through serial commands |
| **Visualization / HRI** | `meshcat_visualizer.py`, `braille_pipeline_node.py` | Show 3-D model and tactile interface for human-robot interaction |

---

## 🔩 2. ROS 2 Package Structure

```

src/
├── mycobot_interface/        # Serial communication, diagnostics, gripper control
├── controller_node/          # Real-time execution of joint or Cartesian trajectories
├── trajectory_optimizer/     # Crocoddyl + Pinocchio optimal control
├── belief_planner/           # Particle-filter or POMDP-based state estimation
├── meshcat_visualizer/       # 3-D web-based visualization
└── braille_interface/        # NVBlox → Braille tactile map conversion

```

Each node communicates via ROS 2 topics and services using standard message types:
- `sensor_msgs/JointState`, `geometry_msgs/PoseStamped`, `std_msgs/Float64MultiArray`, etc.  
- Parameters are loaded via YAML for modular launch configuration.

---

## 🪞 3. Perception Layer

### RealSense D455 (Lower Mount)
Publishes RGB, depth, and camera info topics at ~30 Hz:
```

/camera/color/image_raw
/camera/depth/image_rect_raw
/camera/aligned_depth_to_color/image_raw

```

### ZED 2i (Upper Mount)
Publishes stereo + IMU data for higher-level mapping and SLAM:
```

/zed2i/left/image_rect_color
/zed2i/right/image_rect_color
/zed2i/imu/data

```

### NVBlox Integration
The NVBlox node fuses depth and pose data into a 3-D voxel map:
```

/nvblox_node/mesh
/nvblox_node/static_occupancy_grid

```

Output feeds both visualization and Braille map generation.

---

## 📍 4. State Estimation Layer

Implements **landmark-based particle filtering** and **belief-space estimation**.

| Node | Description |
|------|--------------|
| `belief_planner_node` | Maintains probabilistic belief over robot and landmark states |
| `particle_filter_node` | Sample-based estimation using camera or encoder observations |
| `pose_interpolator_node` | Interpolates pose for high-rate IMU simulation |

This layer ensures robust estimates despite noisy low-cost sensors.

---

## 🚀 5. Planning and Optimization Layer

### Core Components
- **Pinocchio:** Rigid-body dynamics and kinematics  
- **Crocoddyl:** Optimal control solver (Differential Dynamic Programming)  
- **Python interface:** Defines cost functions, constraints, and goal poses

### Typical Workflow
1. Receive current state (from estimation layer).  
2. Define goal pose / trajectory.  
3. Use Crocoddyl solver to compute control sequence minimizing cost.  
4. Send trajectory to controller for execution.

Optional: integrate **Bayesian policy** or **POMDP planning** for uncertainty-aware decisions.

---

## ⚙️ 6. Control Layer

### MyCobot Interface Node
Responsible for communication between ROS 2 and the MyCobot hardware.

Functions:
- Connect to `/dev/mycobot320`
- Send joint angles or velocities
- Query servo states and errors
- Manage gripper and power control

Example topic interface:
```

/mycobot/angles_cmd
/mycobot/angles_state
/mycobot/power_state

````

### Controller Node
- Executes trajectories computed by Crocoddyl  
- Supports feedback (e.g., PID or model-based control)  
- Publishes real-time motion feedback

---

## 🧩 7. Visualization and Human–Robot Interface

### MeshCat Visualizer
- Displays robot joint motion in browser (WebGL)
- Reads URDF via Pinocchio
- Publishes / subscribes to ROS topics for live updates

### Braille Interface
- Converts NVBlox occupancy map into tactile patterns for **Canute 360** Braille display  
- Supports split view (map + grayscale image)
- Enables tactile situational awareness for visually impaired users

---

## 🐳 8. Docker and Environment

All components are containerized for reproducibility.

| Container | Purpose |
|------------|----------|
| `isaac_ros-dev` | Core ROS 2 workspace (Isaac ROS, NVBlox, Visual SLAM) |
| `mycobot-dev` | MyCobot interface, Pinocchio, Crocoddyl, MeshCat |
| `braille-dev` | Tactile interface pipeline and visualization tools |

**Example run command:**
```bash
./scripts/run_dev.sh -d ${ISAAC_ROS_WS} --device /dev/mycobot320 --privileged
````

---

## 🔗 9. Data Flow Summary

```
ZED / RealSense  ─┬─>  NVBlox  ─┬─>  Occupancy Map
                   │             ├─>  Braille Interface (Tactile Output)
                   │             └─>  Belief Planner (State Estimation)
MyCobot Interface ─┘
       ↓
Trajectory Optimizer (Pinocchio + Crocoddyl)
       ↓
Controller Node  →  MyCobot Hardware
       ↓
MeshCat Visualizer / Braille Display
```

---

## 🧩 10. Launch Workflow

Typical integrated launch sequence:

```bash
# 1. Start cameras and mapping
ros2 launch nvblox_examples bringup_realsense.launch.py

# 2. Start robot and controller
ros2 launch mycobot_interface bringup.launch.py

# 3. Start trajectory optimizer
ros2 run trajectory_optimizer optimizer_node.py

# 4. Optional: visualization and Braille output
ros2 run meshcat_visualizer visualize_robot.py
ros2 run braille_interface publish_braille_map.py
```

---

✅ **Result:**
All modules interact within ROS 2 to form a cohesive perception–estimation–control loop, enabling low-cost robots like the MyCobot 320 to perform adaptive, uncertainty-aware, and human-interpretable tasks.

```

---

Would you like me to create a **matching system diagram image (PNG or ASCII block)** showing these module interactions for embedding at the top of this file (e.g., `docs/architecture_overview.png`)?
```
