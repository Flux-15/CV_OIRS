"""
IRS Mirror Auto-Alignment — Camera Calibration Utility
=======================================================
Captures checkerboard images from the webcam and computes
camera intrinsics (matrix + distortion coefficients).

Run standalone:
    python calibration.py
"""

import cv2
import numpy as np
import logging
import config

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(name)s] %(message)s")
logger = logging.getLogger(__name__)


def run_calibration():
    board_size = config.CHECKERBOARD_SIZE
    num_frames = config.CALIBRATION_FRAMES

    # 3D object points (z=0 plane, unit squares)
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)

    obj_points = []   # 3D world points
    img_points = []   # 2D image points

    cap = cv2.VideoCapture(config.CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, config.CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, config.CAMERA_HEIGHT)

    if not cap.isOpened():
        logger.error("Cannot open camera.")
        return

    logger.info(f"Calibration: capture {num_frames} frames of a "
                f"{board_size[0]}x{board_size[1]} checkerboard.")
    logger.info("Press SPACE to capture a frame, 'q' to finish early.")

    captured = 0
    while captured < num_frames:
        ret, frame = cap.read()
        if not ret:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, board_size, None)

        display = frame.copy()
        if found:
            cv2.drawChessboardCorners(display, board_size, corners, found)
            cv2.putText(display, "Checkerboard FOUND - press SPACE to capture",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display, "Searching for checkerboard...",
                        (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.putText(display, f"Captured: {captured}/{num_frames}",
                    (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.imshow("Calibration", display)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord(' ') and found:
            # Refine to sub-pixel accuracy
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria
            )
            obj_points.append(objp)
            img_points.append(corners_refined)
            captured += 1
            logger.info(f"Captured frame {captured}/{num_frames}")

    cap.release()
    cv2.destroyAllWindows()

    if captured < 3:
        logger.error("Need at least 3 frames for calibration. Aborting.")
        return

    logger.info("Computing calibration (this may take a moment)...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None
    )

    if ret:
        np.savez(config.CALIBRATION_FILE,
                 camera_matrix=camera_matrix,
                 dist_coeffs=dist_coeffs,
                 K=camera_matrix,
                 D=dist_coeffs)
        logger.info(f"Calibration saved to {config.CALIBRATION_FILE}")
        logger.info(f"RMS reprojection error: {ret:.4f}")
        logger.info(f"Camera matrix:\n{camera_matrix}")
        logger.info(f"Distortion coefficients: {dist_coeffs.ravel()}")
    else:
        logger.error("Calibration failed.")


if __name__ == "__main__":
    run_calibration()
