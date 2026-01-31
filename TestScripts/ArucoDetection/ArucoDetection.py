import cv2
import cv2.aruco as aruco
import numpy as np
import os


def load_calibration(filename):
    """
    Loads camera matrix and distortion coefficients from a .npz file.
    """
    if not os.path.exists(filename):
        print(f"Error: Calibration file '{filename}' not found.")
        print("Please ensure 'webcam_calibration.npz' is in the same folder.")
        return None, None

    with np.load(filename) as data:
        # Try to access keys, handle potential naming variations
        if 'camera_matrix' in data:
            mtx = data['camera_matrix']
        elif 'mtx' in data:
            mtx = data['mtx']
        else:
            print("Error: Could not find 'camera_matrix' in .npz file.")
            return None, None

        if 'dist_coeffs' in data:
            dist = data['dist_coeffs']
        elif 'dist' in data:
            dist = data['dist']
        else:
            print("Error: Could not find 'dist_coeffs' in .npz file.")
            return None, None

    return mtx, dist


def calculate_midpoint(tvecs):
    """
    Calculates the average (centroid) of a list of translation vectors.
    """
    if len(tvecs) == 0:
        return None

    # Calculate the mean of X, Y, and Z coordinates
    x_avg = np.mean([t[0] for t in tvecs])
    y_avg = np.mean([t[1] for t in tvecs])
    z_avg = np.mean([t[2] for t in tvecs])

    return np.array([x_avg, y_avg, z_avg])


def urc_task_targeting():
    # --- CONFIGURATION ---
    camera_index = 0
    calib_filename = "webcam_calibration.npz"

    # Task Specifics
    # URC Rules: USB markers are 1 x 1 cm
    USB_MARKER_SIZE = 0.01  # 1 cm in meters
    REQUIRED_TAG_COUNT = 4  # User requirement: Look for exactly 4 tags
    TARGET_COLOR = (0, 255, 255)  # Yellow

    # --- SETUP ---

    # 1. Load Calibration
    camera_matrix, dist_coeffs = load_calibration(calib_filename)
    if camera_matrix is None or dist_coeffs is None:
        return  # Exit if calibration fails

    # 2. Setup ArUco
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # 3. Start Camera
    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print(f"Error: Could not open camera {camera_index}")
        return

    print("URC Task Targeting Started.")
    print(f"Using calibration from: {calib_filename}")
    print(f"Looking for exactly {REQUIRED_TAG_COUNT} tags of any ID to define the USB slot...")

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)

            count = len(ids)

            # LOGIC: Only proceed if we see the expected number of tags
            if count == REQUIRED_TAG_COUNT:

                # Estimate pose for ALL detected markers using the USB size
                # Since we don't filter by ID, we assume all 4 visible tags are the USB corners
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners, USB_MARKER_SIZE, camera_matrix, dist_coeffs
                )

                # Extract tvecs (3D positions)
                valid_tvecs = [tvec[0] for tvec in tvecs]

                # Calculate the CENTROID of the 4 markers
                target_point_3d = calculate_midpoint(valid_tvecs)

                if target_point_3d is not None:
                    # --- VISUALIZATION ---

                    # 1. Project the 3D target point back to 2D pixel coordinates
                    imgpts, _ = cv2.projectPoints(
                        np.array([target_point_3d]),
                        np.zeros(3), np.zeros(3),
                        camera_matrix, dist_coeffs
                    )

                    center_x = int(imgpts[0][0][0])
                    center_y = int(imgpts[0][0][1])

                    # 2. Draw the Target Crosshair
                    cv2.circle(frame, (center_x, center_y), 10, TARGET_COLOR, -1)
                    cv2.putText(frame, "USB TARGET", (center_x + 15, center_y),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, TARGET_COLOR, 2)

                    # 3. Draw Distance to Target
                    dist = np.linalg.norm(target_point_3d)
                    cv2.putText(frame, f"Depth: {dist:.3f}m", (center_x + 15, center_y + 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, TARGET_COLOR, 2)

                    # Draw lines from all 4 markers to the center
                    for i in range(count):
                        marker_center = np.mean(corners[i][0], axis=0).astype(int)
                        cv2.line(frame, tuple(marker_center), (center_x, center_y), TARGET_COLOR, 1)

            elif count > 0:
                # Feedback if we see tags but not the right amount
                cv2.putText(frame, f"Tags Visible: {count}/{REQUIRED_TAG_COUNT}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        else:
            cv2.putText(frame, "Searching for USB Markers...", (10, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('URC Task Targeting', frame)
        if cv2.waitKey(1) == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    urc_task_targeting()