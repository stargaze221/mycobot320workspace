
---

# 🧩 MyCobot 320 M5 2022 – Pinocchio + MeshCat Setup Summary

*Date: 2025-10-23*
*Environment: Isaac ROS Dev Container on Jetson AGX Orin (Ubuntu 22.04, ROS 2 Humble)*

---

## 🧱 1️⃣ Goal

Set up a clean simulation environment for the **MyCobot 320 M5 2022** robotic arm using:

* **Pinocchio** for kinematics & URDF parsing
* **MeshCat** for 3-D visualization
* **CasADi** for future optimization and trajectory control

---

## 🧰 2️⃣ Dockerfile Additions

**File:** `Dockerfile.yoonlab`

Key additions made after installing `pymycobot`:

```dockerfile
# --------------------------------------------------
# Install Pinocchio (ROS 2 Humble prebuilt)
# --------------------------------------------------
RUN apt-get update && \
    apt-get install -y --no-install-recommends ros-humble-pinocchio && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# --------------------------------------------------
# Install MeshCat (visualization) and CasADi (optimization)
# --------------------------------------------------
RUN pip install --no-cache-dir \
        meshcat \
        casadi && \
    pip install --no-cache-dir "numpy<2"  # ensure compatibility with Pinocchio
```

✅ Installed successfully

* `pinocchio 2.9.x`
* `meshcat latest`
* `casadi 3.7.x`
* `numpy 1.26.x` (to ensure ROS Humble ABI compatibility)

---

## 🧠 3️⃣ Environment Verification

```bash
python3 -c "import pinocchio, casadi, meshcat, numpy; \
print(pinocchio.__version__, casadi.__version__, numpy.__version__)"
```

All imports succeed without warnings or segmentation faults.

---

## 🤖 4️⃣ Robot Model Visualization Demo

**File:** `hellopinocchio.py`

### Main features

1. Loads URDF from
   `/workspaces/isaac_ros-dev/src/mycobot_ros2/mycobot_description/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf`
2. Applies **0.001 × scale correction** (meshes in mm).
3. Initializes MeshCat visualizer and animates a sinusoidal motion.

### Run

```bash
source /opt/ros/humble/setup.bash
source /workspaces/isaac_ros-dev/install/setup.bash
python3 hellopinocchio.py
```

### Result

* Opens automatically at **[http://127.0.0.1:7000/static/](http://127.0.0.1:7000/static/)**
* Robot appears at correct scale and animates smoothly.
* MeshCat rendering quality noticeably higher than RViz.

---

## 🌐 5️⃣ Observations

| Feature                | Outcome                                              |
| ---------------------- | ---------------------------------------------------- |
| Pinocchio URDF Parsing | ✅ Works (after sourcing ROS env)                     |
| Mesh Scaling           | ✅ Correct (× 0.001)                                  |
| Visualization          | ✅ MeshCat → crisp & responsive                       |
| Performance            | ⚡ Real-time on AGX Orin                              |
| Next Step              | ➡️ Integrate ROS 2 topics for Foxglove visualization |

---

## 🚀 6️⃣ Next Week Plan

| Task                         | Description                                                                                    | Expected Outcome                                              |
| ---------------------------- | ---------------------------------------------------------------------------------------------- | ------------------------------------------------------------- |
| **1. ROS 2 State Publisher** | Finalize `publish_mycobot_state.py` to publish `/joint_states` and `/tf`.                      | Live MyCobot motion visible in **Foxglove Studio** 3-D scene. |
| **2. CasADi Integration**    | Implement a basic trajectory optimization (e.g., reach target pose minimizing joint velocity). | Optimized motion visualized in MeshCat.                       |
| **3. URDF Validation**       | Verify joint limits and link frames against hardware specs.                                    | Accurate kinematic model for control.                         |
| **4. Docker Refinement**     | Add auto-sourcing of ROS setup in `workspace-entrypoint.sh`.                                   | No manual `source` needed each session.                       |
| **5. Documentation Update**  | Extend README with Foxglove integration steps and animation examples.                          | Ready for repository release.                                 |

---

✅ **Status:**

* Pinocchio + MeshCat + CasADi functional
* URDF loads correctly after ROS sourcing
* Mesh scaling resolved programmatically
* Visualization confirmed in browser

Next session → connect this simulation to ROS 2 topics and Foxglove Studio for live visualization.

---