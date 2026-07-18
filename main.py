"""
OIRS 4-Mirror Auto-Alignment System -- Main Entry Point
========================================================
IK-based alignment for a 2x2 grid of pan-tilt mirrors.
Each mirror independently computes its bisection angle via
inverse kinematics to reflect light from Node 1 (TX) to Node 2 (RX).

Pipeline per frame:
    Camera -> ArUco (detect Node 2) -> solvePnP (3D position)
        -> For each mirror:
            desired_normal(mirror_pos, node1, node2) -> IK -> servo angles
        -> ALL command -> serial -> ESP32 -> PCA9685 -> 8 servos

Usage:
    python main.py
Debug window controls:
    q   - Quit
    h   - Home all servos (90, 90)
"""
import sys
import threading
import time
import logging
import cv2
import numpy as np
import config
from camera import CameraCapture
from detection import ArucoDetector, load_calibration
from serial_comm import SerialComm
from control import MirrorController, ControlLoop
from kinematics import compute_servo_angles, inverse_kinematics, kinematic_to_servo_deg
from overlay import draw_debug_overlay


# ─── Logging ──────────────────────────────────────────────────────────
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(name)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
logger = logging.getLogger("main")


# ─── Dry-run serial ──────────────────────────────────────────────────
class DryRunSerial:
    """Dummy serial for testing without ESP32."""
    def send_all_angles(self, angles_flat):
        logger.debug(f"[DRY-RUN] {angles_flat}")
        return True
    def send_angles(self, pan, tilt):
        return True
    def read_response(self):
        return None
    def close(self):
        pass


# ─── Tracking modes ──────────────────────────────────────────────────
MODE_IK_TRACKING = "IK_TRACKING"    # Node 2 detected -> full IK
MODE_NO_NODE2 = "NO_NODE2"          # Node 2 not visible -> hold


# ─── Detection thread ────────────────────────────────────────────────
def detection_thread_func(camera, detector, control_loop, state):
    """
    Detection thread: detect Node 2, run IK for all 4 mirrors,
    feed target angles to the control loop.
    """
    logger.info("Detection thread started (4-mirror IK).")
    node1_pos = config.NODE1_POSITION_WORLD
    mirror_positions = config.MIRROR_POSITIONS

    while state["running"]:
        ok, frame = camera.get_frame()
        if not ok:
            time.sleep(0.005)
            continue

        # Detect markers
        detections = detector.detect(frame)

        # Find Node 1 (TX) and Node 2 (RX)
        node1_det = None
        node2_det = None
        for d in detections:
            if d.marker_id == config.MARKER_ID_NODE_1:
                node1_det = d
            elif d.marker_id == config.MARKER_ID_NODE_2:
                node2_det = d

        if node1_det is not None and node1_det.position_world is not None:
            node1_pos = node1_det.position_world

        with state["lock"]:
            state["all_detections"] = detections
            state["frame"] = frame
            state["node1_pos"] = node1_pos
            state["node1_live"] = (node1_det is not None)

        if node2_det is not None and node2_det.position_world is not None:
            node2_pos = node2_det.position_world

            # Run IK for each mirror independently
            target_pans = []
            target_tilts = []
            all_n_des = []

            for i in range(config.NUM_MIRRORS):
                pan_deg, tilt_deg, n_des, theta_p, theta_t = compute_servo_angles(
                    mirror_positions[i], node1_pos, node2_pos,
                    config.PAN_CALS[i], config.TILT_CALS[i],
                )
                target_pans.append(pan_deg)
                target_tilts.append(tilt_deg)
                all_n_des.append(n_des)

            control_loop.set_target_angles(target_pans, target_tilts)

            with state["lock"]:
                state["mode"] = MODE_IK_TRACKING
                state["node2_pos"] = node2_pos
                state["target_pans"] = target_pans
                state["target_tilts"] = target_tilts
                state["all_n_des"] = all_n_des

        else:
            control_loop.set_no_detection()
            with state["lock"]:
                state["mode"] = MODE_NO_NODE2
                state["node2_pos"] = None
                state["target_pans"] = None
                state["target_tilts"] = None
                state["all_n_des"] = None

        time.sleep(0.01)

    logger.info("Detection thread stopped.")



# ─── Main ─────────────────────────────────────────────────────────────
def main():
    logger.info("=" * 60)
    logger.info("  OIRS 4-Mirror Auto-Alignment (IK Mode)")
    logger.info("=" * 60)
    logger.info(f"Mirrors:        {config.NUM_MIRRORS} (2x2 grid)")
    logger.info(f"Marker size:    {config.MARKER_SIZE_M * 1000:.0f} mm")
    logger.info(f"Node1 (TX):     {config.NODE1_POSITION_WORLD} (default fallback, live ArUco ID 1 overrides)")
    logger.info(f"Camera pos:     {config.CAMERA_POSITION_WORLD}")
    for i in range(config.NUM_MIRRORS):
        logger.info(f"Mirror {i} pos:   {config.MIRROR_POSITIONS[i]}  "
                    f"pan_sign={config.PAN_CALS[i]['sign']}  "
                    f"tilt_sign={config.TILT_CALS[i]['sign']}")
    logger.info(f"Rate limit:     {config.RATE_LIMIT_DEG_PER_S} deg/s")
    logger.info(f"Servo range:    [{config.SERVO_SAFE_MIN}, {config.SERVO_SAFE_MAX}]")
    logger.info("")

    # ── 1. Camera ─────────────────────────────────────────────────────
    camera = CameraCapture()
    camera.start()

    # ── 2. Detection ──────────────────────────────────────────────────
    calib_data = load_calibration()
    if calib_data is None:
        logger.error("Camera calibration is REQUIRED for IK mode.")
        logger.error("Run 'python calibration.py' first.")
        camera.stop()
        sys.exit(1)

    detector = ArucoDetector(calibration_data=calib_data)

    # ── 3. Serial to ESP32 ────────────────────────────────────────────
    serial_comm = SerialComm()
    try:
        serial_comm.connect()
    except ConnectionError as e:
        logger.warning(f"Serial connection failed: {e}")
        logger.warning("==> Running in DRY-RUN mode.")
        serial_comm = DryRunSerial()

    # ── 4. Controller + loop ──────────────────────────────────────────
    controller = MirrorController()
    control_loop = ControlLoop(controller, serial_comm)
    control_loop.start()

    # ── 5. Shared state ──────────────────────────────────────────────
    state = {
        "running": True,
        "lock": threading.Lock(),
        "frame": None,
        "all_detections": [],
        "mode": MODE_NO_NODE2,
        "node1_pos": config.NODE1_POSITION_WORLD,
        "node1_live": False,
        "node2_pos": None,
        "target_pans": None,
        "target_tilts": None,
        "all_n_des": None,
    }

    # ── 6. Start detection thread ─────────────────────────────────────
    det_thread = threading.Thread(
        target=detection_thread_func,
        args=(camera, detector, control_loop, state),
        daemon=True,
        name="DetectionThread",
    )
    det_thread.start()

    # ── 7. Main thread: OpenCV debug window ───────────────────────────
    logger.info("Debug window active. Press 'q' to quit.")
    logger.info("Show Node 2 marker (ID 2) to the camera to start tracking.")
    fps_timer = time.monotonic()
    fps_count = 0
    fps_display = 0.0

    try:
        while True:
            with state["lock"]:
                frame = state.get("frame")
                if frame is not None:
                    frame = frame.copy()

            if frame is not None:
                frame = draw_debug_overlay(frame, state, control_loop, controller,
                                           camera_matrix=detector.camera_matrix,
                                           dist_coeffs=detector.dist_coeffs)
                fps_count += 1
                now = time.monotonic()
                if now - fps_timer >= 1.0:
                    fps_display = fps_count / (now - fps_timer)
                    fps_count = 0
                    fps_timer = now
                cv2.putText(frame, f"FPS: {fps_display:.1f}",
                            (frame.shape[1] - 140, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("OIRS 4-Mirror Alignment (IK)", frame)

            key = cv2.waitKey(16) & 0xFF
            if key == ord('q'):
                logger.info("Quit requested by user.")
                break
            elif key == ord('h'):
                for i in range(config.NUM_MIRRORS):
                    controller.pans[i] = config.SERVO_HOME_PAN
                    controller.tilts[i] = config.SERVO_HOME_TILT
                    controller.target_pans[i] = config.SERVO_HOME_PAN
                    controller.target_tilts[i] = config.SERVO_HOME_TILT
                logger.info(">> All servos homed to "
                            f"({config.SERVO_HOME_PAN}, {config.SERVO_HOME_TILT})")

    except KeyboardInterrupt:
        logger.info("Interrupted (Ctrl+C).")
    finally:
        logger.info("Shutting down...")
        state["running"] = False
        control_loop.stop()
        camera.stop()
        if hasattr(serial_comm, 'close'):
            serial_comm.close()
        cv2.destroyAllWindows()
        logger.info("Shutdown complete.")


if __name__ == "__main__":
    main()