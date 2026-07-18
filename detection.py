"""
OIRS Mirror Auto-Alignment -- ArUco Detection + 3D Pose Estimation
=================================================================
Detects ArUco markers in camera frames, computes centroid pixel
coordinates, and estimates 3D marker positions using solvePnP.
Transforms to world frame including camera position offset.
"""

import logging
from collections import namedtuple
import cv2
import numpy as np
import config

logger = logging.getLogger(__name__)

# ─── Data structures ──────────────────────────────────────────────────

Detection = namedtuple("Detection", [
    "marker_id",      # int -- ArUco marker ID
    "centroid",        # (u, v) pixel coordinates
    "corners",         # np.ndarray shape (4, 2) -- corner points
    "position_cam",    # np.ndarray (3,) -- 3D position in camera frame (meters), or None
    "position_world",  # np.ndarray (3,) -- 3D position in world frame (meters), or None
])


# ─── ArUco Detector ───────────────────────────────────────────────────

class ArucoDetector:
    """Detects ArUco markers and estimates their 3D positions via solvePnP."""

    def __init__(self, dictionary_id=None, calibration_data=None):
        dict_id = dictionary_id if dictionary_id is not None else config.ARUCO_DICTIONARY
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera calibration for undistortion + solvePnP
        self.camera_matrix = None
        self.dist_coeffs = None
        self.has_calibration = False
        if calibration_data is not None:
            self.camera_matrix = calibration_data.get("camera_matrix")
            if self.camera_matrix is None:
                self.camera_matrix = calibration_data.get("K")
            self.dist_coeffs = calibration_data.get("dist_coeffs")
            if self.dist_coeffs is None:
                self.dist_coeffs = calibration_data.get("D")
            if self.camera_matrix is not None and self.dist_coeffs is not None:
                self.has_calibration = True
                logger.info("Camera calibration loaded -- pose estimation enabled.")
            else:
                logger.warning("Incomplete calibration data -- pose estimation DISABLED.")

        # 3D object points for a square marker (marker's own frame, Z=0 plane)
        half = config.MARKER_SIZE_M / 2.0
        self.obj_points = np.array([
            [-half,  half, 0.0],
            [ half,  half, 0.0],
            [ half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)

        # Camera-to-world transform
        self.R_cam_to_world = config.R_CAM_TO_WORLD
        self.camera_pos_world = config.CAMERA_POSITION_WORLD

    def detect(self, frame):
        """
        Detect all ArUco markers in the frame and estimate 3D positions.

        Returns:
            List of Detection namedtuples for every recognized marker.
        """
        if self.has_calibration:
            frame = cv2.undistort(frame, self.camera_matrix, self.dist_coeffs)

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)

        detections = []
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corners = corners_list[i][0]
                centroid = corners.mean(axis=0)

                position_cam = None
                position_world = None
                if self.has_calibration:
                    position_cam = self._estimate_position(corners)
                    if position_cam is not None:
                        # World = R * cam_pos + camera_offset
                        position_world = (self.R_cam_to_world @ position_cam
                                          + self.camera_pos_world)

                detections.append(Detection(
                    marker_id=int(marker_id),
                    centroid=(float(centroid[0]), float(centroid[1])),
                    corners=corners,
                    position_cam=position_cam,
                    position_world=position_world,
                ))
        return detections

    def _estimate_position(self, corners):
        """
        Estimate 3D position of a single marker using solvePnP.

        Returns:
            np.ndarray (3,) -- translation vector in camera frame (meters),
            or None on failure.
        """
        image_points = corners.reshape(4, 1, 2).astype(np.float64)
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            image_points,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if success:
            return tvec.flatten()
        else:
            logger.warning("solvePnP failed for a marker.")
            return None


# ─── Calibration I/O ──────────────────────────────────────────────────

def load_calibration(path=None):
    """Load camera calibration from a .npz file."""
    path = path or config.CALIBRATION_FILE
    try:
        data = np.load(path)
        camera_matrix = data["camera_matrix"] if "camera_matrix" in data else data["K"]
        dist_coeffs = data["dist_coeffs"] if "dist_coeffs" in data else data["D"]
        logger.info(f"Camera calibration loaded from {path}")
        return {
            "camera_matrix": camera_matrix,
            "dist_coeffs": dist_coeffs,
        }
    except FileNotFoundError:
        logger.warning(f"No calibration file at '{path}' -- "
                        "Run 'python calibration.py' to create one.")
        return None
    except KeyError as e:
        logger.error(f"Calibration file '{path}' is missing expected keys: {e}")
        return None
    except Exception as e:
        logger.error(f"Error loading calibration from '{path}': {e}")
        return None
