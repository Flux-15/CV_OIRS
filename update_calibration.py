"""
Update camera calibration with new matrix and distortion coefficients
"""
import numpy as np
import config

# New calibration data
camera_matrix = np.array([
    [1.06239381e+03, 0.00000000e+00, 9.44474045e+02],
    [0.00000000e+00, 1.06885655e+03, 5.34847950e+02],
    [0.00000000e+00, 0.00000000e+00, 1.00000000e+00]
], dtype=np.float64)

dist_coeffs = np.array([[0.00463077, -0.08432499, -0.00023444, 0.00094999, 0.05336193]], dtype=np.float64)

# Save to calibration file
np.savez(config.CALIBRATION_FILE,
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs,
         K=camera_matrix,
         D=dist_coeffs)

print(f"✓ Calibration updated in {config.CALIBRATION_FILE}")
print(f"\nCamera matrix:\n{camera_matrix}")
print(f"\nDistortion coefficients:\n{dist_coeffs}")
print(f"\nReprojection error: 0.1954")

# Verify by loading it back
data = np.load(config.CALIBRATION_FILE)
print(f"\n✓ Verification - loaded matrix:\n{data['camera_matrix']}")
print(f"\n✓ Verification - loaded distortion:\n{data['dist_coeffs']}")
