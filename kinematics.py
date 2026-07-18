"""
OIRS Mirror Alignment — Kinematic Model
=========================================
Implements the forward/inverse kinematics for a 2-DOF AZ-EL pan-tilt
mirror gimbal, plus the law-of-reflection bisector for computing the
desired mirror normal.

Reference: User-provided kinematic model (§1–§9).

Coordinate convention (world frame {W}):
    ẑ = vertical (pan rotation axis)
    x̂, ŷ = horizontal, right-handed
    Pan  θ_p rotates about ẑ  (azimuth)
    Tilt θ_t rotates about the tilt shaft (elevation)

Mirror body-frame normal at home (θ_p=0, θ_t=0):
    n̂₀ = [0, 1, 0]ᵀ  (faces +ŷ when both servos are at kinematic zero)
"""

import numpy as np
import logging

logger = logging.getLogger(__name__)


# ─── Forward Kinematics ──────────────────────────────────────────────

def forward_normal(theta_p, theta_t):
    """
    Forward kinematics: servo kinematic angles → mirror normal (unit vector).

    R(θ_p, θ_t) = Rz(θ_p) · Rx(θ_t)
    n̂ = R · [0, 1, 0]ᵀ

    Args:
        theta_p: Pan angle in radians.
        theta_t: Tilt angle in radians.

    Returns:
        np.array of shape (3,) — unit mirror normal in world frame.
    """
    return np.array([
        -np.sin(theta_p) * np.cos(theta_t),
         np.cos(theta_p) * np.cos(theta_t),
         np.sin(theta_t),
    ])


# ─── Desired Normal (Law of Reflection) ──────────────────────────────

def desired_normal(p_mirror, p_tx, p_rx):
    """
    Compute the mirror normal that reflects light from TX to RX.

    By the law of reflection, the required normal bisects the unit
    direction vectors from the mirror to each node:
        n̂_des = normalize(ŝ + r̂)
    where ŝ = direction to TX, r̂ = direction to RX.

    Args:
        p_mirror: np.array (3,) — mirror position in world frame (meters).
        p_tx:     np.array (3,) — transmitter (Node 1) position (meters).
        p_rx:     np.array (3,) — receiver (Node 2) position (meters).

    Returns:
        np.array (3,) — unit desired normal in world frame.
    """
    s = p_tx - p_mirror
    s_norm = np.linalg.norm(s)
    if s_norm < 1e-9:
        logger.warning("TX coincident with mirror — cannot compute normal.")
        return np.array([0.0, 1.0, 0.0])
    s = s / s_norm

    r = p_rx - p_mirror
    r_norm = np.linalg.norm(r)
    if r_norm < 1e-9:
        logger.warning("RX coincident with mirror — cannot compute normal.")
        return np.array([0.0, 1.0, 0.0])
    r = r / r_norm

    n = s + r
    n_mag = np.linalg.norm(n)
    if n_mag < 1e-9:
        # TX and RX are exactly opposite from the mirror — degenerate
        logger.warning("TX and RX are exactly opposite — bisector undefined.")
        return np.array([0.0, 1.0, 0.0])

    return n / n_mag


# ─── Inverse Kinematics ──────────────────────────────────────────────

def inverse_kinematics(n_des):
    """
    Closed-form inverse kinematics: desired normal → kinematic angles.

    Given n̂_des = (nx, ny, nz):
        θ_t = arcsin(nz)
        θ_p = atan2(-nx, ny)

    This is exact for a 2-DOF AZ-EL gimbal. Only degenerate at
    θ_t = ±90° (mirror vertical — not physically reachable).

    Args:
        n_des: np.array (3,) — unit desired normal in world frame.

    Returns:
        (theta_p, theta_t) — kinematic angles in radians.
    """
    nx, ny, nz = n_des
    theta_t = np.arcsin(np.clip(nz, -1.0, 1.0))
    theta_p = np.arctan2(-nx, ny)
    return theta_p, theta_t


# ─── Servo Angle Mapping ─────────────────────────────────────────────

def kinematic_to_servo_deg(theta_rad, cal):
    """
    Convert a kinematic angle (radians) to a servo mechanical angle (degrees).

    From the model (§7):
        φ = φ_c + s · θ
    where φ is the servo mechanical angle, φ_c is the center offset,
    s is the sign (+1 or -1), and θ is in degrees.

    Args:
        theta_rad: Kinematic angle in radians.
        cal: dict with keys 'phi_c' (float, degrees), 'sign' (+1 or -1),
             'Phi' (float, degrees — total servo range for clamping).

    Returns:
        Servo mechanical angle in degrees, clamped to [0, Phi].
    """
    theta_deg = np.degrees(theta_rad)
    servo_deg = cal['phi_c'] + cal['sign'] * theta_deg
    return float(np.clip(servo_deg, 0.0, cal['Phi']))


def servo_to_kinematic_rad(servo_deg, cal):
    """
    Inverse of kinematic_to_servo_deg: servo mechanical angle → kinematic angle.

    θ = (φ - φ_c) / s

    Args:
        servo_deg: Servo mechanical angle in degrees.
        cal: dict with 'phi_c', 'sign', 'Phi'.

    Returns:
        Kinematic angle in radians.
    """
    theta_deg = (servo_deg - cal['phi_c']) / cal['sign']
    return np.radians(theta_deg)


# ─── Camera-to-World Coordinate Transform ────────────────────────────

def cam_to_world(position_cam, R_cam_to_world):
    """
    Transform a 3D position from OpenCV camera frame to world frame.

    OpenCV camera frame: x=right, y=down, z=forward
    World frame:         x=right, y=forward, z=up

    Args:
        position_cam: np.array (3,) — position in camera frame (meters).
        R_cam_to_world: np.array (3,3) — rotation matrix from camera to world.

    Returns:
        np.array (3,) — position in world frame (meters).
    """
    return R_cam_to_world @ position_cam


# ─── Full Pipeline (convenience) ─────────────────────────────────────

def compute_servo_angles(p_mirror_world, p_tx_world, p_rx_world,
                         pan_cal, tilt_cal):
    """
    Full IK pipeline: node positions → servo mechanical angles.

    1. Compute desired normal (bisector law)
    2. Inverse kinematics → kinematic angles
    3. Map to servo mechanical angles

    Args:
        p_mirror_world: np.array (3,) — mirror position in world frame.
        p_tx_world:     np.array (3,) — TX (Node 1) position in world frame.
        p_rx_world:     np.array (3,) — RX (Node 2) position in world frame.
        pan_cal:  dict — pan servo calibration.
        tilt_cal: dict — tilt servo calibration.

    Returns:
        (pan_servo_deg, tilt_servo_deg, n_des, theta_p_rad, theta_t_rad)
    """
    n_des = desired_normal(p_mirror_world, p_tx_world, p_rx_world)
    theta_p, theta_t = inverse_kinematics(n_des)
    pan_servo_deg = kinematic_to_servo_deg(theta_p, pan_cal)
    tilt_servo_deg = kinematic_to_servo_deg(theta_t, tilt_cal)
    return pan_servo_deg, tilt_servo_deg, n_des, theta_p, theta_t


# ─── Standalone verification ─────────────────────────────────────────

if __name__ == "__main__":
    print("=" * 60)
    print("  Kinematics Module — Verification")
    print("=" * 60)

    # §6 round-trip test: FK → IK should recover original angles
    rng = np.random.default_rng(42)
    N = 200_000
    theta_p_all = rng.uniform(-np.pi / 2, np.pi / 2, N)
    theta_t_all = rng.uniform(-np.pi / 2, np.pi / 2, N)

    max_err = 0.0
    for tp, tt in zip(theta_p_all, theta_t_all):
        n = forward_normal(tp, tt)
        tp_rec, tt_rec = inverse_kinematics(n)
        err = max(abs(tp - tp_rec), abs(tt - tt_rec))
        max_err = max(max_err, err)

    print(f"FK->IK round-trip over {N:,} random angle pairs:")
    print(f"  Max reconstruction error: {max_err:.2e} rad")
    assert max_err < 1e-10, f"Round-trip error too large: {max_err}"
    print("  PASSED")

    # section 5 reflection test: 45° geometry
    p_mirror = np.array([0.0, 0.0, 0.0])
    p_tx = np.array([-1.0, 1.0, 0.0])   # 45° left
    p_rx = np.array([1.0, 1.0, 0.0])     # 45° right
    n = desired_normal(p_mirror, p_tx, p_rx)
    print(f"\nReflection test (TX 45° left, RX 45° right):")
    print(f"  Desired normal: {n}")
    # Incidence angle = angle between -s and n
    s_hat = (p_tx - p_mirror) / np.linalg.norm(p_tx - p_mirror)
    r_hat = (p_rx - p_mirror) / np.linalg.norm(p_rx - p_mirror)
    inc_angle = np.degrees(np.arccos(np.clip(np.dot(s_hat, n), -1, 1)))
    ref_angle = np.degrees(np.arccos(np.clip(np.dot(r_hat, n), -1, 1)))
    print(f"  Incidence angle: {inc_angle:.3f}°")
    print(f"  Reflection angle: {ref_angle:.3f}°")
    assert abs(inc_angle - ref_angle) < 1e-10, "Angles not equal!"
    print("  PASSED (incidence = reflection)")
    print()
