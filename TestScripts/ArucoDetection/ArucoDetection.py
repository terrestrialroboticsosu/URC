import cv2
import cv2.aruco as aruco
import numpy as np
import math
# ====================== CONFIG ======================

DESIRED_DISTANCE = 2.0
KP_FORWARD = 50
KP_TURN = 50

TAG_SIZE = 0.2  # meters (IMPORTANT: must match real tag)

CALIB_FILE = "/home/jay-patel/Documents/URC/TestScripts/ArucoDetection/webcam_calibration.yaml"
CAMERA_INDEX = 33

SPIN_SPEED = 50

# ====================== LOAD YAML ======================

def load_yaml(filename):
    fs = cv2.FileStorage(filename, cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        print("Failed to open calibration file")
        return None, None

    cam = fs.getNode("camera_matrix").mat()
    dist = fs.getNode("dist_coeffs").mat()
    fs.release()

    return cam, dist

# ====================== EULER ======================

def get_euler(rvec):
    # Convert rotation vector to rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])

    singular = sy < 1e-6

    if not singular:
        roll  = math.atan2(R[2,1], R[2,2])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = math.atan2(R[1,0], R[0,0])
    else:
        roll  = math.atan2(-R[1,2], R[1,1])
        pitch = math.atan2(-R[2,0], sy)
        yaw   = 0

    # Convert radians to degrees
    roll  = math.degrees(roll)
    pitch = math.degrees(pitch)
    yaw   = math.degrees(yaw)

    return roll, pitch, yaw

# ====================== MOTOR ======================

def clamp(v, mn, mx):
    return max(min(v, mx), mn)

def send_motor_command(left, right):
    left = -left

    left_conv  = int(clamp(left + 100, 0, 200))
    right_conv = int(clamp(right + 100, 0, 200))

    print(f"[CMD] L:{left_conv} R:{right_conv}")

# ====================== MAIN ======================

def main():

    camera_matrix, dist_coeffs = load_yaml(CALIB_FILE)
    if camera_matrix is None:
        return

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    params = aruco.DetectorParameters()

    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)

    if not cap.isOpened():
        print("Camera failed")
        return

    print("=== MANUAL solvePnP MODE ===")

    stopped = False

    # ✅ DEFINE REAL WORLD MARKER CORNERS (CRITICAL)
    half = TAG_SIZE / 2
    object_points = np.array([
        [-half, -half, 0],
        [ half, -half, 0],
        [ half,  half, 0],
        [-half,  half, 0]
    ], dtype=np.float32)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        detector = aruco.ArucoDetector(aruco_dict, params)
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:

            for i in range(len(ids)):

                img_points = corners[i].reshape((4,2)).astype(np.float32)

                # ===== SOLVE PNP =====
                success, rvec, tvec = cv2.solvePnP(
                    object_points,
                    img_points,
                    camera_matrix,
                    dist_coeffs
                )

                if not success:
                    continue

                x = tvec[0][0]
                z = tvec[2][0]

                roll, pitch, yaw = get_euler(rvec)

                # ===== DRAW =====
                cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rvec, tvec, TAG_SIZE/2)

                cx, cy = img_points[0].astype(int)

                cv2.putText(frame, f"ID:{ids[i][0]}", (cx, cy),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                cv2.putText(frame, f"x:{x:.2f} z:{z:.2f}m",
                            (cx, cy-20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                cv2.putText(frame, f"YAW:{yaw:.1f}",
                            (cx, cy-40), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)

                # ===== STOP LOGIC =====
                if z <= DESIRED_DISTANCE:
                    if not stopped:
                        print("STOPPED AT 2m")
                        send_motor_command(0, 0)
                        stopped = True
                    continue
                else:
                    stopped = False

                # ===== CONTROL =====
                dist_error = z - DESIRED_DISTANCE

                forward = clamp(KP_FORWARD * dist_error, -100, 100)
                turn = clamp(KP_TURN * x, -100, 100)

                left  = clamp(forward - turn, -100, 100)
                right = clamp(forward + turn, -100, 100)

                send_motor_command(left, right)

        else:
            stopped = False
            send_motor_command(SPIN_SPEED, -SPIN_SPEED)

        cv2.imshow("Manual solvePnP", frame)

        if cv2.waitKey(1) == ord('q'):
            send_motor_command(0, 0)
            break

    cap.release()
    cv2.destroyAllWindows()

# ====================== ENTRY ======================

if __name__ == "__main__":
    main()