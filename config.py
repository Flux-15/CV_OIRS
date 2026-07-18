"""
OIRS 4-Mirror Auto-Alignment System -- Configuration
=====================================================
All tunable parameters in one place.
4 mirrors in a 2x2 grid, each with independent pan-tilt servos.
"""

import cv2
import numpy as np

# ─── Camera ───────────────────────────────────────────────────────────
CAMERA_INDEX = 0              # cv2.VideoCapture device index
CAMERA_WIDTH = 1280           # Requested frame width (pixels)
CAMERA_HEIGHT = 720           # Requested frame height (pixels)

# ─── ArUco ────────────────────────────────────────────────────────────
ARUCO_DICTIONARY = cv2.aruco.DICT_6X6_50
MARKER_SIZE_M = 0.080      # Physical marker side length in METERS (80 mm = 0.080 m)

# Marker role assignments (both Node 1 and Node 2 can be detected dynamically)
MARKER_ID_NODE_1 = 1          # Fixed transmitter node (TX) -- detected via ArUco ID 1
MARKER_ID_NODE_2 = 2          # Dynamic receiver node (RX) -- detected via ArUco ID 2
ALL_MARKER_IDS = [1, 2]

# ─── Assembly Geometry (World Frame, meters) ──────────────────────────
# World frame origin = center of the 4-mirror assembly
#   x = right,  y = forward (toward nodes),  z = up
#
# 2x2 grid layout (viewed from behind, camera on top):
#
#         [M0]  [M1]     <-- top row
#           --------
#           | 10cm |
#           --------
#         [M2]  [M3]     <-- bottom row
#          9cm apart
#
NUM_MIRRORS = 4

MIRROR_POSITIONS = np.array([
    [-0.045,  0.0,  +0.05],   # M0: top-left
    [+0.045,  0.0,  +0.05],   # M1: top-right
    [-0.045,  0.0,  -0.05],   # M2: bottom-left
    [+0.045,  0.0,  -0.05],   # M3: bottom-right
], dtype=np.float64)

# Camera is 20cm above the center of the assembly, looking forward
CAMERA_POSITION_WORLD = np.array([0.0, 0.0, 0.20], dtype=np.float64)

# ─── Camera-to-World Coordinate Transform ─────────────────────────────
# OpenCV camera frame: x=right, y=DOWN, z=FORWARD
# World frame {W}:     x=right, y=FORWARD, z=UP
#
# Rotation (no translation -- translation added separately):
#     world_x =  cam_x
#     world_y =  cam_z
#     world_z = -cam_y
R_CAM_TO_WORLD = np.array([
    [ 1.0,  0.0,  0.0],
    [ 0.0,  0.0,  1.0],
    [ 0.0, -1.0,  0.0],
], dtype=np.float64)

# ─── Node Positions (World Frame, meters) ─────────────────────────────
# Node 1 (TX): Default fallback position. When Marker ID 1 (TX ArUco) is visible
# in the camera frame, its live detected solvePnP 3D position overrides this!
NODE1_POSITION_WORLD = np.array([0.0, 1.25, 0.0], dtype=np.float64)

# Node 2 (RX): DYNAMIC -- detected by camera each frame
# (no config entry -- comes from solvePnP at runtime)

# ─── PCA9685 Channel Map ─────────────────────────────────────────────
# Each mirror has 2 channels: [pan_channel, tilt_channel]
CHANNEL_MAP = [
    (0, 1),   # M0: pan=CH0, tilt=CH1
    (2, 3),   # M1: pan=CH2, tilt=CH3
    (4, 5),   # M2: pan=CH4, tilt=CH5
    (6, 7),   # M3: pan=CH6, tilt=CH7
]

# ─── Servo Calibration (Kinematic -> Mechanical) ─────────────────────
# Per-servo constants from the kinematic model:
#   phi_c: servo mechanical angle at kinematic theta=0 (degrees)
#   Phi:   total servo mechanical range (degrees)
#   sign:  +1 or -1 (flip if servo moves wrong direction)
#
# All 4 mirrors start with identical calibration.
# Tune individually if needed by replacing entries.
_DEFAULT_PAN_CAL  = {'phi_c': 90.0, 'Phi': 180.0, 'sign': +1}
_DEFAULT_TILT_CAL = {'phi_c': 90.0, 'Phi': 180.0, 'sign': +1}

PAN_CALS  = [_DEFAULT_PAN_CAL.copy()  for _ in range(NUM_MIRRORS)]
TILT_CALS = [_DEFAULT_TILT_CAL.copy() for _ in range(NUM_MIRRORS)]

# ─── Servo Limits ─────────────────────────────────────────────────────
SERVO_SAFE_MIN = 30.0         # Safe operating minimum (degrees)
SERVO_SAFE_MAX = 150.0        # Safe operating maximum (degrees)
SERVO_HOME_PAN = 90.0         # Home/startup pan angle (mechanical degrees)
SERVO_HOME_TILT = 90.0        # Home/startup tilt angle (mechanical degrees)

# ─── Control Loop ─────────────────────────────────────────────────────
CONTROL_LOOP_HZ = 20          # Control loop rate (Hz)
RATE_LIMIT_DEG_PER_S = 60.0   # Max servo speed (degrees/second)
MARKER_LOST_THRESHOLD = 15    # Consecutive missed frames before "LOST"

# ─── Serial (ESP32) ──────────────────────────────────────────────────
SERIAL_BAUD = 115200
SERIAL_PORT = "auto"          # "auto" = scan & handshake, or e.g. "COM3"
SERIAL_TIMEOUT = 1.0          # Read timeout (seconds)

# ─── Camera Calibration ──────────────────────────────────────────────
CALIBRATION_FILE = "camera_calibration.npz"
CHECKERBOARD_SIZE = (9, 6)
CALIBRATION_FRAMES = 20
