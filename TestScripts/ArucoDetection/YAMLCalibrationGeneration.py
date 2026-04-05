import cv2
import numpy as np
import glob
import os

# --- Configuration ---
CHECKERBOARD_SIZE = (7, 10) # (8 squares - 1, 11 squares - 1)
IMAGE_DIR = 'calibration_images'
CALIBRATION_FILE = 'webcam_calibration.yml' # Output for C++
SQUARE_SIZE_MM = 17 
# --- End Configuration ---

print("Starting calibration...")

criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

objp = np.zeros((CHECKERBOARD_SIZE[0] * CHECKERBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD_SIZE[0], 0:CHECKERBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE_MM

objpoints = [] # 3D points
imgpoints = [] # 2D points

images = glob.glob(os.path.join(IMAGE_DIR, '*.png'))
if not images:
    print(f"Error: No images found in {IMAGE_DIR}.")
    exit()

img_shape = None

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    if img_shape is None:
        img_shape = gray.shape[::-1]

    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)
    else:
        print(f"Could not find corners in {fname}")


if not objpoints:
    print("Error: No valid images found for calibration.")
    exit()

print(f"Calibrating using {len(objpoints)} valid images...")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, img_shape, None, None
)

if not ret:
    print("Calibration failed!")
    exit()

print("Calibration successful!")
print("\nCamera Matrix (mtx):\n", camera_matrix)
print("\nDistortion Coefficients (dist):\n", dist_coeffs)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
    mean_error += error
print(f"\nTotal re-projection error: {mean_error / len(objpoints)}")


# --- NEW/CORRECTED SAVING METHOD ---
# Save to a .yml file that C++ can read
fs = cv2.FileStorage(CALIBRATION_FILE, cv2.FILE_STORAGE_WRITE)
fs.write("camera_matrix", camera_matrix)
fs.write("dist_coeffs", dist_coeffs)
fs.release() # Release the file

print(f"\nCalibration data saved to {CALIBRATION_FILE} (for C++)")