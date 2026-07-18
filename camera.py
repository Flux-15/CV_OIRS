"""
IRS Mirror Auto-Alignment — Camera Capture Module
==================================================
Threaded camera capture: always exposes the latest frame, no queue.

Backend swap guide (future Raspberry Pi migration):
    Subclass CameraCapture and override _open(), _read(), _release()
    with picamera2 equivalents. The threading/interface stays the same.
"""

import threading
import time
import logging
import cv2
import config

logger = logging.getLogger(__name__)


class CameraCapture:
    """
    Continuously grabs frames from a camera in a background thread.
    Always stores only the most recent frame — old frames are dropped.
    """

    def __init__(self, device_index=None, width=None, height=None):
        self.device_index = device_index if device_index is not None else config.CAMERA_INDEX
        self.width = width or config.CAMERA_WIDTH
        self.height = height or config.CAMERA_HEIGHT

        self._cap = None
        self._frame = None
        self._frame_lock = threading.Lock()
        self._running = False
        self._thread = None
        self._frame_count = 0

    # ─── Backend methods (override these for picamera2) ───────────────

    def _open(self):
        """Open the camera. Override for alternative backends."""
        self._cap = cv2.VideoCapture(self.device_index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Minimize latency
        if not self._cap.isOpened():
            raise RuntimeError(f"Cannot open camera at index {self.device_index}")
        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        logger.info(f"Camera opened: requested {self.width}x{self.height}, "
                     f"actual {actual_w}x{actual_h}")

    def _read(self):
        """Read one frame. Override for alternative backends.
        Returns: (success: bool, frame: np.ndarray or None)
        """
        return self._cap.read()

    def _release(self):
        """Release the camera. Override for alternative backends."""
        if self._cap:
            self._cap.release()

    # ─── Thread loop ──────────────────────────────────────────────────

    def _capture_loop(self):
        logger.info("Camera capture thread started.")
        while self._running:
            ret, frame = self._read()
            if ret:
                with self._frame_lock:
                    self._frame = frame
                    self._frame_count += 1
            else:
                logger.warning("Camera read failed — retrying.")
                time.sleep(0.01)
        logger.info("Camera capture thread stopped.")

    # ─── Public API ───────────────────────────────────────────────────

    def start(self):
        """Start the background capture thread."""
        if self._running:
            logger.warning("Camera already running.")
            return
        self._open()
        self._running = True
        self._thread = threading.Thread(
            target=self._capture_loop, daemon=True, name="CameraThread"
        )
        self._thread.start()

    def stop(self):
        """Stop the capture thread and release the camera."""
        self._running = False
        if self._thread:
            self._thread.join(timeout=3.0)
        self._release()
        logger.info("Camera stopped and released.")

    def get_frame(self):
        """
        Get the most recent frame (non-blocking, zero-copy-ish).

        Returns:
            (True, frame_copy) if a frame is available,
            (False, None)      if no frame captured yet.
        """
        with self._frame_lock:
            if self._frame is not None:
                return True, self._frame.copy()
            return False, None

    @property
    def is_running(self):
        return self._running

    @property
    def frame_count(self):
        return self._frame_count


# ─── Standalone test ──────────────────────────────────────────────────
if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO,
                        format="%(asctime)s [%(name)s] %(message)s")
    cam = CameraCapture()
    cam.start()
    print("Camera test — press 'q' to quit.")
    try:
        while True:
            ok, frame = cam.get_frame()
            if ok:
                cv2.imshow("Camera Test", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()
