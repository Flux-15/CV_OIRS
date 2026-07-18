"""
OIRS 4-Mirror Auto-Alignment -- Serial Communication Module
============================================================
Handles the serial protocol between the laptop and ESP32.

Protocol (4-mirror / 8-servo):
    Laptop -> ESP32:  ALL:p0,t0,p1,t1,p2,t2,p3,t3\\n
    ESP32 -> Laptop:  OK:ALL:p0,t0,p1,t1,p2,t2,p3,t3\\n
    Handshake:        PING\\n -> PONG\\n
"""

import time
import logging
import serial
import serial.tools.list_ports
import config

logger = logging.getLogger(__name__)


class SerialComm:
    """Serial communication with the ESP32 servo controller."""

    def __init__(self, port=None, baud=None, timeout=None):
        self.port = port or config.SERIAL_PORT
        self.baud = baud or config.SERIAL_BAUD
        self.timeout = timeout or config.SERIAL_TIMEOUT
        self.ser = None

    def connect(self):
        """
        Open the serial connection.
        If port is "auto", scans all COM ports and tries a PING/PONG handshake.
        """
        if self.port == "auto":
            detected = self._auto_detect()
            if detected is None:
                raise ConnectionError(
                    "Could not auto-detect ESP32 on any serial port.\n"
                    "  -> Is the ESP32 plugged in and flashed with the sketch?\n"
                    "  -> Try setting SERIAL_PORT manually in config.py."
                )
            self.port = detected

        logger.info(f"Connecting to ESP32 on {self.port} @ {self.baud} baud...")
        self.ser = serial.Serial(self.port, self.baud, timeout=self.timeout)
        time.sleep(2.0)  # Wait for ESP32 reset after serial open
        self._flush_input()
        logger.info(f"Serial connected: {self.port}")

    def _auto_detect(self):
        """Scan COM ports and handshake with PING/PONG to find the ESP32."""
        ports = serial.tools.list_ports.comports()
        logger.info(f"Auto-detect: scanning {len(ports)} serial port(s)...")
        for p in ports:
            desc = p.description or "unknown"
            logger.info(f"  Trying {p.device} ({desc})...")
            try:
                s = serial.Serial(p.device, self.baud, timeout=2.0)
                time.sleep(2.0)
                s.reset_input_buffer()
                s.write(b"PING\n")
                time.sleep(0.5)
                response = s.readline().decode(errors="ignore").strip()
                if response == "PONG":
                    logger.info(f"  -> ESP32 found on {p.device}")
                    s.close()
                    return p.device
                else:
                    logger.info(f"  -> Got '{response}' (not PONG), skipping.")
                s.close()
            except Exception as e:
                logger.debug(f"  -> Error: {e}")
        return None

    def _flush_input(self):
        """Discard stale data in the serial input buffer."""
        if self.ser and self.ser.in_waiting:
            self.ser.reset_input_buffer()

    def send_all_angles(self, angles_flat):
        """
        Send all servo angles in one command (4 mirrors = 8 values).

        Args:
            angles_flat: list of 8 floats [p0, t0, p1, t1, p2, t2, p3, t3]
                         in mechanical servo degrees.

        Returns:
            True if sent successfully, False on error.
        """
        if not self.ser or not self.ser.is_open:
            logger.error("Serial not connected -- cannot send.")
            return False

        values = ",".join(f"{a:.1f}" for a in angles_flat)
        cmd = f"ALL:{values}\n"
        try:
            self.ser.write(cmd.encode())
            logger.debug(f"TX: {cmd.strip()}")
            return True
        except serial.SerialException as e:
            logger.error(f"Serial write error: {e}")
            return False

    def send_angles(self, pan_deg, tilt_deg):
        """
        Legacy: send a single pan/tilt command (backward compatibility).
        Sends as ALL with all mirrors set to the same angles.
        """
        flat = [pan_deg, tilt_deg] * config.NUM_MIRRORS
        return self.send_all_angles(flat)

    def read_response(self):
        """Non-blocking read of a response line from the ESP32."""
        if not self.ser or not self.ser.is_open:
            return None
        try:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode(errors="ignore").strip()
                if line:
                    logger.debug(f"RX: {line}")
                    return line
        except serial.SerialException as e:
            logger.error(f"Serial read error: {e}")
        return None

    def close(self):
        """Close the serial connection cleanly."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            logger.info("Serial connection closed.")


# ─── Standalone test ──────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    print("Serial comm test -- connecting to ESP32...")
    comm = SerialComm()
    try:
        comm.connect()
        print("Sending all mirrors to home (90, 90)...")
        home = [90.0, 90.0] * config.NUM_MIRRORS
        comm.send_all_angles(home)
        time.sleep(0.5)
        resp = comm.read_response()
        print(f"Response: {resp}")
    except ConnectionError as e:
        print(f"Connection failed: {e}")
    finally:
        comm.close()
