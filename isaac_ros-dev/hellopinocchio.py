#!/usr/bin/env python3
"""
Hello MyCobot + MeshCat Demo (Fixed for mycobot_ros2 path)
Loads your MyCobot 320 M5 2022 URDF and displays it in a MeshCat 3D viewer.
"""

import time
from pathlib import Path
import numpy as np
import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer

# -----------------------------------------------------------
# 🦾 Load URDF model (corrected path)
# -----------------------------------------------------------
urdf_path = Path("/workspaces/isaac_ros-dev/src/mycobot_ros2/mycobot_description/urdf/mycobot_320_m5_2022/mycobot_320_m5_2022.urdf")
assert urdf_path.exists(), f"❌ URDF not found at {urdf_path}"

# Package directory to resolve `package://` URIs
package_dirs = ["/workspaces/isaac_ros-dev/src/mycobot_ros2/mycobot_description/"]

# Build Pinocchio models (geometry, visuals)
model, collision_model, visual_model = pin.buildModelsFromUrdf(str(urdf_path), package_dirs)

# --- Apply scale correction to all visual & collision geometry (mm → m)
scale_factor = 0.001

for geom in visual_model.geometryObjects:
    geom.meshScale = geom.meshScale * scale_factor

for geom in collision_model.geometryObjects:
    geom.meshScale = geom.meshScale * scale_factor

print(f"✅ Applied visual/collision scale correction: ×{scale_factor}")

data, collision_data, visual_data = pin.createDatas(model, collision_model, visual_model)
print(f"✅ Loaded model: {model.name} with {model.nq} DoF")

# -----------------------------------------------------------
# 🧠 Initialize MeshCat visualizer
# -----------------------------------------------------------
viz = MeshcatVisualizer(model, collision_model, visual_model)
viz.initViewer(open=True)  # open browser if available
viz.loadViewerModel()
viz.display(pin.neutral(model))
print("✅ MeshCat viewer initialized.")
print("👉 Open this URL in your browser if not auto-opened:")
print(viz.viewer.url())

# -----------------------------------------------------------
# 🎬 Animate simple sinusoidal motion
# -----------------------------------------------------------
q0 = pin.neutral(model)
t = 0.0
dt = 0.05
print("🎞️  Animating joints... (Ctrl+C to stop)")

try:
    while True:
        q = q0.copy()
        for i in range(model.nq):
            q[i] += 0.3 * np.sin(t + i)
        viz.display(q)
        time.sleep(dt)
        t += dt
except KeyboardInterrupt:
    print("🛑 Animation stopped.")
