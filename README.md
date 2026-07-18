# OIRS 4-Mirror Auto-Alignment System (CV_OIRS)

> **Computer-Vision-based IK auto-alignment for a 2×2 pan-tilt mirror array — Intelligent Reflecting Surface (IRS) testbed**

---

## Table of Contents

- [Overview](#overview)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Project Structure](#project-structure)
- [Setup & Installation](#setup--installation)
  - [1. Flash ESP32 Firmware](#1-flash-esp32-firmware)
  - [2. Camera Calibration](#2-camera-calibration)
  - [3. Assembly Geometry Configuration](#3-assembly-geometry-configuration)
  - [4. Servo Calibration](#4-servo-calibration)
- [Usage](#usage)
  - [Running the System](#running-the-system)
  - [Keyboard Controls](#keyboard-controls)
  - [Dry-Run Mode](#dry-run-mode)
- [Technical Details](#technical-details)
  - [Coordinate Frames](#coordinate-frames)
  - [Kinematics Model](#kinematics-model)
  - [Detection Pipeline](#detection-pipeline)
  - [Control Loop](#control-loop)
  - [Serial Protocol](#serial-protocol)
  - [HUD Overlay](#hud-overlay)
- [Configuration Reference](#configuration-reference)
- [Troubleshooting](#troubleshooting)
- [Future Work](#future-work)

---

## Overview

This project implements an **automated mirror alignment system** for an Optical Intelligent Reflecting Surface (OIRS) testbed. The system uses a webcam to detect **ArUco markers** placed on a transmitter (TX, Node 1) and receiver (RX, Node 2), computes the required mirror orientations using the **law of reflection** and **inverse kinematics (IK)**, and drives **8 servos** (4 mirrors × 2 axes) via an ESP32 + PCA9685 to reflect signals from TX to RX.

### Key Features

- **4 independent pan-tilt mirrors** arranged in a 2×2 grid
- **Real-time ArUco marker tracking** with 3D pose estimation via `solvePnP`
- **Closed-form inverse kinematics** for 2-DOF AZ-EL pan-tilt gimbals
- **Law-of-reflection bisector** computation for optimal mirror normal
- **Rate-limited servo control** (60°/s max) for smooth, safe movement
- **Rich HUD overlay** with optical paths, telemetry table, and mirror grid visualizer
- **Auto-detection** of ESP32 via PING/PONG handshake
- **Dry-run mode** for testing without hardware

---

## System Architecture

```
┌─────────────────┐     ┌──────────────────────────┐     ┌───────────────────┐     ┌──────────────┐
│ Camera Thread   │────▶│ Detection Thread          │────▶│ Control Loop      │────▶│ ESP32 +      │
│ (camera.py)     │     │ (detection.py +           │     │ Thread (20 Hz)    │     │ PCA9685      │
│ Latest frame    │     │  kinematics.py)           │     │ (control.py)      │     │ 8× MG90      │
│ only, no queue  │     │ ArUco → solvePnP → IK     │     │ Rate-limited      │     │ servos       │
└─────────────────┘     └──────────────────────────┘     │ serial_comm.py    │     └──────────────┘
       │                          │                      └───────────────────┘
       ▼                          ▼                              │
┌──────────────────────────────────────────────────────────────────────┐
│ Main Thread: OpenCV Debug Window (overlay.py)                        │
│ HUD overlay • optical paths • telemetry table • mirror visualizer    │
└──────────────────────────────────────────────────────────────────────┘
```

**Per-frame pipeline:**

1. **Camera thread** continuously captures and holds only the latest frame (zero-latency)
2. **Detection thread** grabs the frame → detects ArUco markers → runs `solvePnP` for 3D pose → transforms camera-frame coordinates to world-frame
3. **Node 1 (TX):** uses live ArUco position if visible, otherwise falls back to configured default position
4. **Node 2 (RX):** must be dynamically detected each frame
5. **IK computation:** for each of 4 mirrors, computes desired mirror normal (law-of-reflection bisector), then closed-form IK to get pan/tilt kinematic angles, mapped to servo degrees
6. **Control loop** at 20 Hz: rate-limits movement (60°/s), clamps to safe range, sends `ALL:p0,t0,...,p3,t3` command over serial
7. **ESP32** receives command, drives all 8 servos via PCA9685 PWM, sends confirmation
8. **Main thread** composites the HUD overlay onto the video frame and displays it

---

## Hardware Requirements

| Component | Specification |
|---|---|
| **Camera** | USB webcam (e.g., Logitech), 720p (1280×720) |
| **Microcontroller** | ESP32 development board |
| **Servo Driver** | PCA9685 16-channel I²C PWM driver (address `0x48`) |
| **Servos** | 8× MG90 micro servos |
| **Mirrors** | 4 mirrors mounted on pan-tilt gimbals, 2×2 grid |
| **ArUco Markers** | DICT_6X6_50, IDs 1 (TX) and 2 (RX), 80 mm physical size |
| **Calibration Board** | Checkerboard pattern, 9×6 inner corners |
| **Power Supply** | External 5V for PCA9685 + servos |

### Wiring

| Connection | Pin |
|---|---|
| PCA9685 SDA | ESP32 GPIO 21 |
| PCA9685 SCL | ESP32 GPIO 22 |
| PCA9685 I²C Address | `0x48` |
| USB Serial | 115200 baud |

### Mirror Channel Map

| Mirror | Position | Pan Channel | Tilt Channel |
|--------|----------|-------------|--------------|
| M0 | Top-Left | CH 0 | CH 1 |
| M1 | Top-Right | CH 2 | CH 3 |
| M2 | Bottom-Left | CH 4 | CH 5 |
| M3 | Bottom-Right | CH 6 | CH 7 |

### Assembly Geometry (default)

The 2×2 mirror grid is centered at the world origin, with 9 cm horizontal and 10 cm vertical spacing:

| Mirror | World Position (m) |
|--------|-------------------|
| M0 (top-left) | (-0.045, 0.0, +0.05) |
| M1 (top-right) | (+0.045, 0.0, +0.05) |
| M2 (bottom-left) | (-0.045, 0.0, -0.05) |
| M3 (bottom-right) | (+0.045, 0.0, -0.05) |

Camera is mounted 20 cm above the assembly center: `(0.0, 0.0, 0.20)`.

---

## Software Dependencies

### Python (3.8+)

```bash
pip install opencv-python opencv-contrib-python pyserial numpy
```

| Package | Purpose |
|---------|---------|
| `opencv-python` | Camera capture, image processing, UI |
| `opencv-contrib-python` | ArUco marker detection |
| `pyserial` | Serial communication with ESP32 |
| `numpy` | Linear algebra, coordinate transforms |

### Arduino / ESP32

- **Arduino IDE** or **PlatformIO**
- **ESP32 Board Package** (Boards Manager)
- **Adafruit PWM Servo Driver Library** (`Adafruit_PWMServoDriver`)

---

## Project Structure

```
CV_OIRS/
├── config.py                   # Central configuration (all tunable parameters)
├── camera.py                   # Threaded webcam capture (latest-frame-only)
├── detection.py                # ArUco detection + solvePnP 3D pose estimation
├── kinematics.py               # FK/IK for 2-DOF AZ-EL pan-tilt gimbals
├── control.py                  # Rate-limited servo controller + control loop thread
├── serial_comm.py              # Serial communication with ESP32 (ALL: protocol)
├── overlay.py                  # HUD & debug overlay (optical paths, telemetry, mirror grid)
├── main.py                     # Entry point — orchestrates all threads
├── calibration.py              # Interactive camera calibration utility
├── update_calibration.py       # Quick calibration update with hardcoded values
├── camera_calibration.npz      # Saved camera intrinsics (camera matrix + distortion)
├── esp32_servo_control/
│   └── esp32_servo_control.ino # ESP32 firmware for PCA9685 servo control
├── overlay_test.png            # Test screenshot of HUD overlay
└── overlay_test_id1.png        # Test screenshot with ID 1 detection
```

---

## Setup & Installation

### 1. Flash ESP32 Firmware

1. Open `esp32_servo_control/esp32_servo_control.ino` in Arduino IDE
2. Install the **Adafruit PWM Servo Driver Library** from Library Manager
3. Select your ESP32 board and COM port
4. Upload the firmware
5. Open Serial Monitor at **115200 baud** — you should see `READY` followed by initial servo positions

### 2. Camera Calibration

Camera calibration is **required** for accurate 3D pose estimation.

**Option A: Interactive calibration (recommended)**
```bash
python calibration.py
```
- Hold a 9×6 checkerboard pattern in front of the camera
- Press **SPACE** to capture a frame (collect at least 3, ideally 20 frames at various angles)
- Press **Q** to finish and compute calibration
- Results are saved to `camera_calibration.npz`

**Option B: Quick update with known values**
```bash
python update_calibration.py
```
Uses hardcoded calibration values from a previous session (RMS error ~0.1954).

### 3. Assembly Geometry Configuration

Edit `config.py` to match your physical setup:

- `MIRROR_POSITIONS` — 4×3 numpy array of mirror positions in world frame (meters)
- `CAMERA_POSITION_WORLD` — camera position relative to mirror assembly center
- `R_CAM_TO_WORLD` — rotation matrix from OpenCV camera frame to world frame
- `NODE1_POSITION_WORLD` — fallback TX position (used when ArUco ID 1 is not visible)
- `MARKER_SIZE_M` — physical size of ArUco markers (default: 80 mm)

### 4. Servo Calibration

In `config.py`, adjust per-servo calibration if mirrors move in the wrong direction or range:

```python
PAN_CALS = [
    {"phi_c": 90.0, "Phi": 180.0, "sign": +1},  # Mirror 0 pan
    {"phi_c": 90.0, "Phi": 180.0, "sign": +1},  # Mirror 1 pan
    {"phi_c": 90.0, "Phi": 180.0, "sign": +1},  # Mirror 2 pan
    {"phi_c": 90.0, "Phi": 180.0, "sign": +1},  # Mirror 3 pan
]
```

- `phi_c` — servo center angle (degrees)
- `Phi` — servo total range (degrees)
- `sign` — `+1` or `-1` to reverse direction

---

## Usage

### Running the System

```bash
python main.py
```

The system will:
1. Start the camera
2. Load camera calibration (exits if not found)
3. Auto-detect the ESP32 via serial PING/PONG handshake
4. If no ESP32 found, fall back to **dry-run mode** (no servos, just visualization)
5. Open the debug window with the live HUD overlay

### Keyboard Controls

| Key | Action |
|-----|--------|
| `Q` | Quit |
| `H` | Home all servos (90° pan, 90° tilt) |

### Dry-Run Mode

If no ESP32 is connected, the system automatically enters dry-run mode — all IK computations and overlay visualizations run normally, but servo commands are logged instead of sent. This is useful for testing the computer vision pipeline without hardware.

---

## Technical Details

### Coordinate Frames

- **World Frame {W}:** x = right, y = forward (toward TX), z = up. Origin at mirror assembly center.
- **Camera Frame {C}:** x = right, y = down, z = forward (OpenCV convention).
- **Transform:** `R_CAM_TO_WORLD` rotates camera-frame vectors into world-frame.

### Kinematics Model

The system uses a **2-DOF Azimuth-Elevation (AZ-EL) pan-tilt gimbal** model:

**Forward Kinematics (FK):**
$$\hat{n} = R_z(\theta_p) \cdot R_x(\theta_t) \cdot \hat{n}_0$$

where $\hat{n}_0 = [0, 1, 0]^T$ (home normal) giving:

$$\hat{n} = [-\sin\theta_p \cos\theta_t,\; \cos\theta_p \cos\theta_t,\; \sin\theta_t]$$

**Desired Mirror Normal (Law of Reflection):**
$$\hat{n}_{des} = \text{normalize}(\hat{s} + \hat{r})$$

where $\hat{s}$ = unit vector toward TX, $\hat{r}$ = unit vector toward RX.

**Inverse Kinematics (IK):**
$$\theta_t = \arcsin(n_z), \quad \theta_p = \text{atan2}(-n_x, n_y)$$

The `kinematics.py` module includes a standalone verification that runs 200,000 random FK→IK round-trip tests (error < 10⁻¹⁰ rad).

### Detection Pipeline

1. Frame captured by threaded camera (latest frame only, no queue latency)
2. ArUco markers detected using `cv2.aruco.ArucoDetector` with `DICT_6X6_50`
3. Frame optionally undistorted using camera calibration
4. 3D pose estimated via `cv2.solvePnP` with `SOLVEPNP_IPPE_SQUARE`
5. Camera-frame position transformed to world-frame: `p_world = R_cam_to_world · p_cam + cam_pos_world`

### Control Loop

- **Update rate:** 20 Hz (configurable via `CONTROL_LOOP_HZ`)
- **Rate limiting:** 60°/s maximum slew rate per axis per mirror
- **Safe range:** 30°–150° (hard clamp)
- **Home position:** 90° pan, 90° tilt
- **Marker lost threshold:** 15 consecutive frames without detection → servos hold position
- **Status reporting:** `LOCKED` (all mirrors within 0.5° of target), `TRACKING`, or `MARKER LOST`

### Serial Protocol

**PC → ESP32:**
```
ALL:p0,t0,p1,t1,p2,t2,p3,t3\n
```
where `p0..p3` are pan angles and `t0..t3` are tilt angles (in degrees).

**ESP32 → PC:**
```
OK:ALL:p0,t0,p1,t1,p2,t2,p3,t3\n
```

**Handshake:**
```
PING\n  →  PONG\n
```

**Legacy (backward-compatible):**
```
PAN:<float>,TILT:<float>\n
```
Broadcasts the same angle to all mirrors.

### HUD Overlay

The overlay renders 5 sections on the video frame:
1. **Optical paths** — 3D-projected TX/RX positions with bisector line and incidence/reflection angles
2. **Top dashboard** — System title + status badge (LOCKED/TRACKING/LOST)
3. **Left telemetry panel** — TX→RX path length, per-mirror target/actual/error/status table
4. **Right mirror visualizer** — 2×2 graphical grid showing pan deflection dials
5. **Bottom reference bar** — Keyboard shortcuts and node info

---

## Configuration Reference

All configuration is centralized in `config.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `CAMERA_INDEX` | 0 | Camera device index |
| `CAMERA_WIDTH / HEIGHT` | 1280 × 720 | Capture resolution |
| `ARUCO_DICTIONARY` | `DICT_6X6_50` | ArUco dictionary type |
| `MARKER_SIZE_M` | 0.080 | Physical marker size (meters) |
| `MARKER_ID_NODE_1` | 1 | TX marker ID |
| `MARKER_ID_NODE_2` | 2 | RX marker ID |
| `NUM_MIRRORS` | 4 | Number of mirrors |
| `CONTROL_LOOP_HZ` | 20 | Control loop rate |
| `RATE_LIMIT_DEG_PER_S` | 60.0 | Maximum servo slew rate |
| `SERVO_SAFE_MIN / MAX` | 30 / 150 | Servo angle hard limits |
| `SERVO_HOME_PAN / TILT` | 90 / 90 | Home position |
| `MARKER_LOST_THRESHOLD` | 15 | Frames before "MARKER LOST" |
| `SERIAL_BAUD` | 115200 | Serial baud rate |
| `SERIAL_PORT` | `"auto"` | COM port (auto-detect if `"auto"`) |

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| `camera_calibration.npz not found` | Run `python calibration.py` or `python update_calibration.py` |
| Mirror moves wrong direction | Change `sign` to `-1` in `PAN_CALS` or `TILT_CALS` in `config.py` |
| ESP32 not detected | Check USB connection, ensure firmware is flashed and `READY` appears on serial |
| ArUco not detected | Ensure markers are DICT_6X6_50, 80 mm, well-lit, and in camera frame |
| Servos jitter | Ensure external 5V power to PCA9685 (not USB power) |
| Low FPS | Reduce `CAMERA_WIDTH/HEIGHT` or `CONTROL_LOOP_HZ` |

---

## Future Work

- **Raspberry Pi 5 migration** — `camera.py` is designed for this: subclass `CameraCapture` and override `_open()`, `_read()`, `_release()` for `picamera2`
- **Multi-RX support** — extend to track multiple receiver nodes simultaneously
- **Closed-loop feedback** — measure reflected signal strength for fine-tuning
- **Web dashboard** — replace OpenCV window with browser-based UI

---

*Project developed during internship — OIRS 4-Mirror Auto-Alignment System*
