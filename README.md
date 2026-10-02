# RC Gamma Macao

Firmware and software for a small-size robot-soccer robot built around a Raspberry Pi 5 (high-level control, vision, lidar) paired with a Raspberry Pi Pico (low-level motor/dribbler control). The robot tracks the ball and gates with a camera, localizes using a lidar, and exchanges live position data (x, y, yaw) with a teammate robot over Bluetooth.

## Repository structure

- **`RC_pico/`** — PlatformIO C++ firmware for the Raspberry Pi Pico: motor control, PID regulators, encoder reading, CAN bus communication (via the `mcp_can` library), and the dribbler mechanism.
- **`Pi5_files/`** — Python scripts that run on the Raspberry Pi 5: camera-based color/ball/gate detection using OpenCV HSV thresholds, lidar interfacing, UART/USB position sharing, and a Bluetooth RFCOMM system (`bt_coords.py`, `bt_setup.py`, and related test scripts) for exchanging coordinates between two robots.
- **`scripts/`** — utility scripts for flashing firmware, generating pin headers, and working with config files.
- **`config/`** — JSON configuration (PID gains, motor speeds, vision HSV ranges, etc.).
- **`device/`** — JSON files identifying each robot/device (id, role, paired-device addresses).

## Getting started

### Pico firmware

The `RC_pico/` firmware is a [PlatformIO](https://platformio.org/) project. Open it in PlatformIO (CLI or the VS Code extension) to build and flash it to the Pico; see `scripts/flash.py` for a flashing helper.

### Raspberry Pi 5 software

The scripts in `Pi5_files/` run directly on the Pi 5 with Python 3 and require OpenCV for vision and PyBluez (or similar) for Bluetooth.

### Inter-robot Bluetooth link

For setting up and testing the Bluetooth coordinate-sharing link between two robots, see the step-by-step recipe in [`Pi5_files/BT_QUICK.txt`](Pi5_files/BT_QUICK.txt).
