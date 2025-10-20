
---

````markdown
# MyCobot 320 Udev Rule Setup

This document explains how to configure a **udev rule** so the MyCobot 320’s serial device is always recognized as `/dev/mycobot320`, even after reboots or USB re-plugs.

---

## 1. Identify the Device

Plug in your MyCobot and check connected USB devices:

```bash
lsusb
````

Typical entry for the MyCobot 320:

```
Bus 001 Device 007: ID 1a86:55d4 QinHeng Electronics USB Single Serial
```

---

## 2. Verify Vendor and Product IDs

Run the following command to inspect the attributes of your serial device (replace `/dev/ttyACM0` if needed):

```bash
udevadm info -a -n /dev/ttyACM0 | grep -E "idVendor|idProduct|manufacturer|product|serial"
```

Example output:

```
ATTRS{idVendor}=="1a86"
ATTRS{idProduct}=="55d4"
ATTRS{product}=="USB Single Serial"
ATTRS{serial}=="5901001607"
```

---

## 3. Create the Udev Rule

Create a new rule file:

```bash
sudo nano /etc/udev/rules.d/99-mycobot.rules
```

Paste the following rule:

```bash
# MyCobot 320 (QinHeng CH340 USB-Serial)
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4", ATTRS{serial}=="5901001607", MODE="0666", GROUP="dialout", SYMLINK+="mycobot320"
```

> The serial number uniquely identifies your device so the rule remains valid even with multiple CH340 adapters connected.

---

## 4. Apply and Test the Rule

Reload and trigger udev:

```bash
sudo udevadm control --reload-rules
sudo udevadm trigger
```

Unplug and replug your MyCobot, then verify:

```bash
ls -l /dev/mycobot320
```

Expected output:

```
lrwxrwxrwx 1 root root 7 Oct 19 14:56 /dev/mycobot320 -> ttyACM0
```

---

## 5. Update Your Python Code

After the rule is active, reference the new symlink in your code:

```python
from pymycobot import MyCobot320
import time

mc = MyCobot320('/dev/mycobot320', 115200)
time.sleep(1)
print("Connected to MyCobot 320 successfully!")
```

---

## 6. Notes

* Group `dialout` allows regular users (non-root) to access the serial port.
* If permissions still fail, ensure your user is part of the `dialout` group:

  ```bash
  sudo usermod -aG dialout $USER
  ```

  Then log out and back in.

---

**✅ Result:**
Your MyCobot 320 will always appear as `/dev/mycobot320` with the correct permissions and stable access path for scripts and ROS 2 nodes.

```

---

Would you like me to actually **generate** and attach the `.md` file here (so you can click and download it from this chat)?
```
