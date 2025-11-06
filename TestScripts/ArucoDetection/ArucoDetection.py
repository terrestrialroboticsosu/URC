import cv2
import numpy as np
import cv2.aruco as aruco
import time

# --- Configuration ---
CALIBRATION_FILE = 'webcam_calibration.npz'

# NEW: Set a default marker size to start with (e.g., 10mm)
current_marker_size = 7.0

# Load the Aruco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()

# Load the calibration data
try:
    with np.load(CALIBRATION_FILE) as data:
        camera_matrix = data['mtx']
        dist_coeffs = data['dist']
    print("Calibration data loaded successfully.")
except FileNotFoundError:
    print(f"Error: Calibration file '{CALIBRATION_FILE}' not found.")
    exit()

# --- Start Webcam ---
cap = cv2.VideoCapture(0)
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# NEW: Add instructions for the user
print("Webcam opened. Press 'q' to quit.")
print("Press '1' for 10mm markers")
print("Press '2' for 20mm markers")
print("Press '3' for 40mm markers")

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- NEW: KEY PRESS LOGIC ---
    key = cv2.waitKey(1) & 0xFF

    if key == ord('1'):
        current_marker_size = 10.0
        print(f"Switched to {current_marker_size}mm")
    elif key == ord('2'):
        current_marker_size = 20.0
        print(f"Switched to {current_marker_size}mm")
    elif key == ord('3'):
        current_marker_size = 40.0
        print(f"Switched to {current_marker_size}mm")
    elif key == ord('q'):
        break
    # --- END NEW ---

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and len(ids) > 0:

        # Estimate pose using the NEW dynamic marker size
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            current_marker_size,  # Use the variable
            camera_matrix,
            dist_coeffs
        )

        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            cv2.drawFrameAxes(
                frame,
                camera_matrix,
                dist_coeffs,
                rvecs[i],
                tvecs[i],
                current_marker_size / 2  # Use the variable
            )

            distance = np.linalg.norm(tvecs[i])
            cv2.putText(
                frame,
                f"ID: {ids[i][0]} Dist: {distance:.0f} mm",
                (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 15),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

    # NEW: Display the current active size on the screen
    cv2.putText(
        frame,
        f"Active Size: {current_marker_size}mm (Keys: 1, 2, 3)",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 100),
        2
    )

    cv2.imshow('ArUco Detection (Calibrated)', frame)

cap.release()
cv2.destroyAllWindows()