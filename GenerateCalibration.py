import cv2
import numpy as np
import glob
import os

# --- Configuration ---
CHECKERBOARD_SIZE = (7, 10)  # Must be (8 squares - 1, 11 squares - 1)
IMAGE_DIR = 'calibration_images'
CALIBRATION_FILE = 'webcam_calibration.npz'

# Size of one square on your checkerboard in mm
SQUARE_SIZE_MM = 17
# --- End Configuration ---

print("Starting calibration...")

# Termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare "object points" (3D points in real-world space)
# (0,0,0), (1,0,0), (2,0,0) ... (6,9,0)
objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE_MM  # Scale by square size

# Arrays to store object points and image points from all images
objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

images = glob.glob(os.path.join(IMAGE_DIR, '*.png'))

if not images:
    print(f"Error: No images found in {IMAGE_DIR}. Did you run the capture script?")
    exit()

img_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    if img_shape is None:
        img_shape = gray.shape[::-1]  # (width, height)

    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

    # If found, add object points and refine image points
    if ret:
        objpoints.append(objp)

        # Refine corner locations
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)
    else:
        print(f"Could not find corners in {fname}")

if not objpoints:
    print("Error: No valid images found for calibration. Make sure CHECKERBOARD_SIZE is correct.")
    exit()

print(f"Calibrating using {len(objpoints)} valid images...")

# Perform calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

if not ret:
    print("Calibration failed!")
    exit()

print("Calibration successful!")

# Print the results
print("\nCamera Matrix (mtx):")
print(camera_matrix)
print("\nDistortion Coefficients (dist):")
print(dist_coeffs)

# Calculate re-projection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error

print(f"\nTotal re-projection error: {mean_error / len(objpoints)}")

# Save the calibration data
np.savez(
    CALIBRATION_FILE,
    mtx=camera_matrix,
    dist=dist_coeffs,
    rvecs=rvecs,
    tvecs=tvecs
)

print(f"\nCalibration data saved to {CALIBRATION_FILE}")
