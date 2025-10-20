Here’s a clean, professional revision of your `README.md` that integrates both your original ROS workspace notes and your new **Pinocchio + Crocoddyl research plan**, while keeping it organized and markdown-ready for your GitHub repository:

---

````markdown
# 🦾 mycobot320workspace

This workspace integrates **MyCobot 320** with **ROS 2 Humble**, **Isaac ROS NVBlox**, and a lightweight **optimal control and planning stack** using **Pinocchio + Crocoddyl**.

---

## ⚙️ Environment Setup

### Add to `~/.bashrc`
```bash
export ISAAC_ROS_WS=${HOME}/mycobot320workspace/isaac_ros-dev/
alias isaacrosdev='cd ${ISAAC_ROS_WS}/src/isaac_ros_common && ./scripts/run_dev.sh'
````

### Submodules

This repository uses the following submodules:

* `isaac_ros_common`
* `isaac_ros_nvblox`

Initialize them with:

```bash
git submodule update --init --recursive
```

---

## 🐳 Dockerfile and Dependencies

A custom `Dockerfile.zed` has been added for ZED SDK + ROS integration.
It now includes `rosdep install` support for ROS image builds.

---

## 🎥 Running ZED and NVBlox

### ZED 2i Camera Wrapper

```bash
ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i
```

### ZED + NVBlox Example

```bash
ros2 launch nvblox_examples_bringup zed_example.launch.py camera:=zed2
```

---

## 🧩 Transition Plan — From MoveIt 2 to Pinocchio + Crocoddyl

Instead of relying on **MoveIt 2**, this workspace will evolve toward a **research-grade, uncertainty-aware control framework** using:

* **Pinocchio** → Fast rigid-body kinematics/dynamics
* **Crocoddyl** → Optimal control & trajectory optimization
* **Custom nodes** → For uncertainty reasoning, belief updates, and adaptive control

---

## 🧠 Conceptual Architecture

```
ROS 2 Graph
│
├── /state_estimator_node        → publishes x̂ , Σ  (state + uncertainty)
├── /belief_planner_node         → chooses goal / subgoal based on uncertainty
├── /trajectory_optimizer_node   → runs Crocoddyl (uses Pinocchio dynamics)
├── /controller_node             → tracks Crocoddyl trajectory
└── /mycobot_interface_node      → converts control signals to pymycobot cmds
```

---

## 🛠 Installation (Ubuntu 22.04 + ROS 2 Humble)

```bash
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
                    python3-pip libeigen3-dev libboost-all-dev
pip install numpy scipy matplotlib ipython jupyter

# Pinocchio
pip install pin==2.9.2  # PyPI package name: 'pin'

# Crocoddyl
pip install crocoddyl
```

---

## 🧮 Minimal Python Example

```python
import numpy as np
import pinocchio as pin
import crocoddyl
from pinocchio.utils import zero

# Load model
model_path = "/path/to/mycobot_description"
urdf_path  = model_path + "/urdf/mycobot.urdf"
robot = pin.RobotWrapper.BuildFromURDF(urdf_path, model_path)

# Simple reaching task
model = crocoddyl.ActionModelLQR(robot.model.nq, robot.model.nv)
ddp   = crocoddyl.SolverDDP(model)

x0 = np.zeros(model.ndx)
us_init = [np.zeros(model.nu)] * 20
ddp.solve(x0, us_init, 100)
print("Final state:", ddp.xs[-1])
```

You can later replace the `ActionModelLQR` with a **Pinocchio-based dynamics model** and custom **cost models** (e.g., tracking goals, minimizing torque, or penalizing uncertainty).

---

## 🔬 Research Extensions

| Goal                          | Extension Path                                                              |
| ----------------------------- | --------------------------------------------------------------------------- |
| **Adaptive control**          | Update dynamics or cost terms from online identification.                   |
| **Uncertainty-aware control** | Add covariance or information gain terms to Crocoddyl’s cost.               |
| **Model-based RL**            | Wrap Crocoddyl inside a learning loop to optimize cost weights.             |
| **Real robot execution**      | Feed optimized joint/torque trajectories to a ROS 2 node using `pymycobot`. |

---

## 📂 Suggested Repository Structure

```
mycobot_optimal_control/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── crocoddyl_demo.launch.py
├── src/
│   ├── mycobot_interface_node.py
│   ├── trajectory_optimizer_node.py
│   ├── controller_node.py
│   └── utils/
│       ├── pinocchio_model.py
│       └── cost_models.py
└── urdf/
    └── mycobot.urdf
```

---

## 🚀 Next Steps

1. ✅ Start with a simple **reach-to-pose demo** using Pinocchio + Crocoddyl (simulation only).
2. 🔄 Add your **ROS 2 interface node** to execute trajectories on the real MyCobot.
3. 🧮 Extend to **uncertainty-aware or adaptive** optimal control.
4. 🧠 Integrate with **Bayesian planners** for belief-based exploration.

---

**Author:** Daniel Hyung-Jin Yoon
*Tennessee Technological University — Robotics and Autonomous Systems Lab*

```

---

Would you like me to add a small section on **Docker usage for Pinocchio + Crocoddyl** (so it runs inside your `Dockerfile.zed` container as well)? That would make the workspace self-contained and reproducible.
```
