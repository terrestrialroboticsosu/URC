import cv2
import numpy as np
import cv2.aruco as aruco

# --- Configuration ---
CALIBRATION_FILE = 'webcam_calibration.npz'
current_marker_size = 10.0  # Default size

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
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Disable autofocus
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

print("Webcam opened. Press 'q' to quit.")
print("Press '1' for 10mm markers")
print("Press '2' for 20mm markers")
print("Press '3' for 40mm markers")


# Function to decompose the rotation matrix into Euler angles
def get_euler_angles(rvec):
    # Convert the Rodrigues vector to a rotation matrix
    R, _ = cv2.Rodrigues(rvec)

    # Decompose the rotation matrix
    # This gives angles in degrees
    angles, mtxR, mtxQ, Qx, Qy, Qz = cv2.RQDecomp3x3(R)

    # angles[0] is roll, angles[1] is pitch, angles[2] is yaw
    return angles


while True:
    ret, frame = cap.read()
    if not ret:
        break

    # --- KEY PRESS LOGIC ---
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

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None and len(ids) > 0:
        # Estimate pose using the dynamic marker size
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
            corners,
            current_marker_size,
            camera_matrix,
            dist_coeffs
        )

        aruco.drawDetectedMarkers(frame, corners, ids)

        for i in range(len(ids)):
            # Draw the 3D axes
            cv2.drawFrameAxes(
                frame,
                camera_matrix,
                dist_coeffs,
                rvecs[i],
                tvecs[i],
                current_marker_size / 2
            )

            # Get and display all 6D pose values

            # 1. Get the translation vector components
            tvec = tvecs[i][0]
            pos_x = tvec[0]
            pos_y = tvec[1]
            pos_z = tvec[2]  # This is the distance

            # 2. Get the rotation angles
            rvec = rvecs[i][0]
            euler_angles = get_euler_angles(rvec)
            roll = euler_angles[0]
            pitch = euler_angles[1]
            yaw = euler_angles[2]

            # 3. Display the values on the screen
            # Get the top-left corner for text placement
            text_corner = (int(corners[i][0][0][0]), int(corners[i][0][0][1]) - 15)

            # Create text strings
            id_str = f"ID: {ids[i][0]}"
            pos_str = f"x: {pos_x:4.0f} y: {pos_y:4.0f} z: {pos_z:4.0f} mm"
            rot_str = f"R: {roll:4.0f} P: {pitch:4.0f} Y: {yaw:4.0f} deg"

            # Draw the text strings, stacked vertically
            cv2.putText(frame, id_str, text_corner,
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, pos_str, (text_corner[0], text_corner[1] - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            cv2.putText(frame, rot_str, (text_corner[0], text_corner[1] - 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the current active size on the screen
    cv2.putText(
        frame,
        f"Active Size: {current_marker_size}mm (Keys: 1, 2, 3)",
        (10, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 100),
        2
    )

    cv2.imshow('ArUco 6D Pose Detection', frame)

cap.release()
cv2.destroyAllWindows()