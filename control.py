"""
OIRS 4-Mirror Auto-Alignment -- Control Module
================================================
Rate-limited servo controller for 4 independent mirrors.
Each mirror has its own pan/tilt state and rate limiting.
"""

import threading
import time
import logging
import config

logger = logging.getLogger(__name__)


class MirrorController:
    """
    Rate-limited controller for NUM_MIRRORS independent pan-tilt mirrors.
    Accepts target servo angles from IK and smoothly slews toward them.
    """

    def __init__(self):
        n = config.NUM_MIRRORS
        self.pans  = [config.SERVO_HOME_PAN]  * n
        self.tilts = [config.SERVO_HOME_TILT] * n
        self.target_pans  = [config.SERVO_HOME_PAN]  * n
        self.target_tilts = [config.SERVO_HOME_TILT] * n
        self.lost_count = 0

    def set_target(self, mirror_idx, pan_deg, tilt_deg):
        """Set the target servo angles for one mirror (mechanical degrees)."""
        self.target_pans[mirror_idx] = max(config.SERVO_SAFE_MIN,
                                           min(config.SERVO_SAFE_MAX, pan_deg))
        self.target_tilts[mirror_idx] = max(config.SERVO_SAFE_MIN,
                                            min(config.SERVO_SAFE_MAX, tilt_deg))

    def set_all_targets(self, pan_list, tilt_list):
        """Set target angles for all mirrors at once."""
        for i in range(config.NUM_MIRRORS):
            self.set_target(i, pan_list[i], tilt_list[i])
        self.lost_count = 0

    def mark_lost(self):
        """Increment the lost counter (no detection this frame)."""
        self.lost_count += 1

    def update(self, dt):
        """
        Move all mirrors toward their targets at a rate-limited speed.

        Returns:
            (all_pans, all_tilts, status_str)
        """
        if self.lost_count >= config.MARKER_LOST_THRESHOLD:
            return (list(self.pans), list(self.tilts),
                    f"MARKER LOST ({self.lost_count} frames)")

        if self.lost_count > 0:
            return (list(self.pans), list(self.tilts),
                    f"No detection ({self.lost_count}/{config.MARKER_LOST_THRESHOLD})")

        max_step = config.RATE_LIMIT_DEG_PER_S * dt
        max_err = 0.0

        for i in range(config.NUM_MIRRORS):
            pan_err = self.target_pans[i] - self.pans[i]
            tilt_err = self.target_tilts[i] - self.tilts[i]

            pan_step = max(-max_step, min(max_step, pan_err))
            tilt_step = max(-max_step, min(max_step, tilt_err))

            self.pans[i] += pan_step
            self.tilts[i] += tilt_step

            self.pans[i] = max(config.SERVO_SAFE_MIN,
                               min(config.SERVO_SAFE_MAX, self.pans[i]))
            self.tilts[i] = max(config.SERVO_SAFE_MIN,
                                min(config.SERVO_SAFE_MAX, self.tilts[i]))

            max_err = max(max_err, abs(pan_err), abs(tilt_err))

        if max_err < 0.5:
            status = "LOCKED (all on target)"
        else:
            status = f"TRACKING (max err={max_err:.1f} deg)"

        return list(self.pans), list(self.tilts), status

    def get_flat_angles(self):
        """Return flat list [p0, t0, p1, t1, ...] for serial transmission."""
        result = []
        for i in range(config.NUM_MIRRORS):
            result.append(self.pans[i])
            result.append(self.tilts[i])
        return result


class ControlLoop:
    """
    Runs the control loop in a background thread at a fixed rate.
    Reads target angles from the detection/IK thread, rate-limits all
    4 mirrors, and sends a single ALL command over serial per cycle.
    """

    def __init__(self, controller, serial_comm):
        self.controller = controller
        self.serial_comm = serial_comm
        self.rate_hz = config.CONTROL_LOOP_HZ
        self._running = False
        self._thread = None

        self._lock = threading.Lock()
        self._has_target = False
        self._target_pans = [config.SERVO_HOME_PAN] * config.NUM_MIRRORS
        self._target_tilts = [config.SERVO_HOME_TILT] * config.NUM_MIRRORS
        self._latest_status = ""

    def set_target_angles(self, pan_list, tilt_list):
        """Called by detection thread: set IK-computed targets for all mirrors."""
        with self._lock:
            self._has_target = True
            self._target_pans = list(pan_list)
            self._target_tilts = list(tilt_list)

    def set_no_detection(self):
        """Called by detection thread when Node 2 is not visible."""
        with self._lock:
            self._has_target = False

    def get_status(self):
        """Get the latest control status string."""
        with self._lock:
            return self._latest_status

    def _loop(self):
        logger.info(f"Control loop started at {self.rate_hz} Hz "
                    f"({config.NUM_MIRRORS} mirrors).")
        period = 1.0 / self.rate_hz

        while self._running:
            t_start = time.monotonic()

            with self._lock:
                has_target = self._has_target
                target_pans = list(self._target_pans)
                target_tilts = list(self._target_tilts)

            if has_target:
                self.controller.set_all_targets(target_pans, target_tilts)
            else:
                self.controller.mark_lost()

            all_pans, all_tilts, status = self.controller.update(period)

            with self._lock:
                self._latest_status = status

            # Send all 8 servo angles in one command
            flat = self.controller.get_flat_angles()
            self.serial_comm.send_all_angles(flat)

            angles_str = " ".join(f"M{i}:({all_pans[i]:.1f},{all_tilts[i]:.1f})"
                                  for i in range(config.NUM_MIRRORS))
            logger.info(f"CMD {angles_str} | {status}")

            resp = self.serial_comm.read_response()
            if resp:
                logger.debug(f"ESP32: {resp}")

            elapsed = time.monotonic() - t_start
            sleep_time = period - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

        logger.info("Control loop stopped.")

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._loop, daemon=True, name="ControlLoop"
        )
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
