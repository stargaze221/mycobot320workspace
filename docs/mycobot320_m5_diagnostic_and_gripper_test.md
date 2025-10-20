# 🦾 MyCobot 320 M5 – Diagnostic + Gripper Torque Test

## 🧩 Purpose
A complete Python script for:
- Checking servo power, enable, and error status  
- Clearing overload/torque alarms (error 2)  
- Reading servo temps, currents, and voltages  
- Moving to the home pose  
- Testing and configuring the **Pro gripper torque limit** (for M5 version)  
- Verifying gripper open/close operation

---

## 🧠 Key Notes
- Requires `pymycobot` Python API (`pip install pymycobot`)  
- Connect robot via `/dev/ttyACM0` (USB)  
- For the **Pro gripper**, torque range ≈ 1 – 100  
- Default `gripper_id = 14`  
- External 12 V power must be connected  

---

## 🧰 Full Script (`test_mycobot320_m5.py`)
```python
from pymycobot import MyCobot320
import time

mc = MyCobot320('/dev/ttyACM0', 115200)
time.sleep(1)
print("=== MyCobot 320 M5 Diagnostic + Gripper Torque Test ===")

# ------------------------------------------------------
# Power / Servo enable check
# ------------------------------------------------------
power_state = mc.is_power_on()
print(f"Power on?  {power_state}")
if not power_state:
    print("Powering on servos...")
    mc.power_on()
    time.sleep(2)

all_enabled = mc.is_all_servo_enable()
print(f"All servos enabled?  {all_enabled}")
if not all_enabled:
    print("Enabling all servos...")
    mc.focus_all_servos()
    time.sleep(2)

# ------------------------------------------------------
# Error handling
# ------------------------------------------------------
err = mc.get_error_information()
print(f"Error info: {err}")

if err != 0:
    print("\n⚠️  Detected error code:", err)
    print("Gathering servo diagnostics...")

    temps = mc.get_servo_temps()
    currents = mc.get_servo_currents()
    voltages = mc.get_servo_voltages()
    print(f"Temps (°C):     {temps}")
    print(f"Currents (mA):  {currents}")
    print(f"Voltages (V):   {voltages}")

    print("\nAttempting to clear error...")
    mc.clear_error_information()
    time.sleep(1)
    new_err = mc.get_error_information()
    print(f"Error after clear: {new_err}")

    if new_err != 0:
        print("❌ Error persists — check arm position or physical load.")
    else:
        print("✅ Error cleared successfully. Re-enabling power...")
        mc.power_on()
        time.sleep(2)
else:
    print("✅ No errors detected.")

# ------------------------------------------------------
# Home motion
# ------------------------------------------------------
print("\nAttempting go_home() ...")
mc.go_home()
time.sleep(5)
angles = mc.get_angles()
print("Angles now:", angles)

# ------------------------------------------------------
# Gripper Torque + Motion Test
# ------------------------------------------------------
print("\n=== Testing Pro Gripper Torque Limit ===")
try:
    gripper_id = 14
    torque_value = 30  # 1–100 range typical
    print(f"Setting gripper torque to {torque_value} ...")
    ret = mc.set_pro_gripper_torque(gripper_id, torque_value)
    time.sleep(1)

    if ret == 1 or ret is None:
        current_torque = mc.get_pro_gripper_torque(gripper_id)
        print(f"✅ Gripper torque set successfully. Current torque value: {current_torque}")
    else:
        print(f"⚠️ Failed to set gripper torque (return={ret}). Continuing anyway.")
except Exception as e:
    print("⚠️ Gripper torque control not supported or communication failed.")
    print("Exception:", e)

# ------------------------------------------------------
# Gripper Open/Close test
# ------------------------------------------------------
print("\n=== Testing Gripper Motion ===")
try:
    mc.init_electric_gripper()
    time.sleep(2)
    print("Opening gripper...")
    mc.set_pro_gripper_open()
    time.sleep(2)
    print("Closing gripper...")
    mc.set_pro_gripper_close()
    time.sleep(2)
    print("✅ Gripper open/close test complete.")
except Exception as e:
    print("⚠️ Gripper command failed — check connection or try electric_gripper API.")
    print("Exception:", e)

print("\n=== Diagnostic + Gripper Torque Test Finished ===")
```

---

## 🧾 Expected Behavior
1. Prints servo status and diagnostics  
2. Clears any error 2 (over-current alarm)  
3. Moves the arm home  
4. Sets gripper torque limit to 30  
5. Opens and closes the gripper once  
6. Ends with diagnostic summary  
