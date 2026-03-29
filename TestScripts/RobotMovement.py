import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import serial

# ====================== CONFIGURATION ======================


time.sleep(45)
DESIRED_DISTANCE = 1.8    
KP_FORWARD = 45           
KP_TURN = 35              
TURN_MULTIPLIER = 1.8     # Increased for more turning "punch"
MIN_DRIVE_POWER = 35      # Increased floor to overcome friction
MAX_SPEED = 100           

# --- MOTOR BIAS ---
# If the left motor is weaker, set this > 1.0 (e.g., 1.25 adds 25% more power to left)
LEFT_MOTOR_BIAS = 10  

TAG_SIZE = 0.145          
TARGET_COLOR = (0, 255, 255)
CALIB_FILENAME = "webcam_calibration.npz"
CAMERA_INDEX = 33         
SPIN_SPEED = 50           

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# ====================== HELPERS ======================

def load_calibration(filename):
    if not os.path.exists(filename):
        print(f"Error: Calibration file '{filename}' not found.")
        return None, None
    with np.load(filename) as data:
        mtx = data.get('camera_matrix') or data.get('mtx')
        dist = data.get('dist_coeffs') or data.get('dist')
        return mtx, dist

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def send_motor_command(left, right, ser):
    # Match existing inversion logic
    left_inv = -int(left)
    right_inv = -int(right)

    left_conv  = int(clamp(left_inv + 100, 0, 200))
    right_conv = int(clamp(right_inv + 100, 0, 200))

    packet = [0xBE, 0xEF, left_conv, right_conv, 100, 100, 100, 100, 100, 100, 0, 0]
    ser.write(bytearray(packet))

# ====================== MAIN ======================

def urc_task_targeting():
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILENAME)
    if camera_matrix is None: return

    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Serial connected")
    except Exception as e:
        print(f"Serial error: {e}"); ser = None

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    while True:
        ret, frame = cap.read()
        if not ret: break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            obj_pts = np.array([[-TAG_SIZE/2, -TAG_SIZE/2, 0],[TAG_SIZE/2, -TAG_SIZE/2, 0],
                                [TAG_SIZE/2, TAG_SIZE/2, 0],[-TAG_SIZE/2, TAG_SIZE/2, 0]], dtype=np.float32)
            img_pts = corners[0].reshape((4,2)).astype(np.float32)
            success, _, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)

            if success:
                z_dist = tvec.flatten()[2]
                # --- EMERGENCY/FINAL STOP LOGIC ---
                if z_dist <= DESIRED_DISTANCE:
                    print(f"!!! TARGET REACHED: {z_dist:.2f}m !!!")
                    if ser:
                        # Send stop command multiple times to ensure it hits
                        for _ in range(5):
                            send_motor_command(0, 0, ser)
                            time.sleep(0.05)
                    break # Breaks loop to close script
                # Centering Logic
                frame_center_x = frame.shape[1] / 2
                tag_center_x = np.mean(img_pts[:, 0])
                centering_error = (tag_center_x - frame_center_x) / frame_center_x

                # 1. Base Forward + Turn
                forward_output = (z_dist - DESIRED_DISTANCE) * KP_FORWARD
                turn_output = (centering_error * KP_TURN) * TURN_MULTIPLIER

                # 2. Differential Mixing
                left_motor = forward_output + turn_output
                right_motor = forward_output - turn_output

                # 3. APPLY LEFT WHEEL BIAS
                left_motor = left_motor * LEFT_MOTOR_BIAS

                # 4. Overcome Friction Floor
                if abs(left_motor) > 1:
                    left_motor = np.sign(left_motor) * max(abs(left_motor), MIN_DRIVE_POWER)
                if abs(right_motor) > 1:
                    right_motor = np.sign(right_motor) * max(abs(right_motor), MIN_DRIVE_POWER)

                # 5. Final Clamp
                left_motor = clamp(left_motor, -MAX_SPEED, MAX_SPEED)
                right_motor = clamp(right_motor, -MAX_SPEED, MAX_SPEED)

                if ser: send_motor_command(left_motor, right_motor, ser)

                # Visuals
                cv2.circle(frame, (int(tag_center_x), int(np.mean(img_pts[:,1]))), 10, TARGET_COLOR, -1)
                
                # Display Motors and Distance in the top left
                info_text = f"L:{int(left_motor)} R:{int(right_motor)} DIST:{z_dist:.2f}m"
                cv2.putText(frame, info_text, (20, 40), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, TARGET_COLOR, 2)
        else:
            if ser: send_motor_command(-SPIN_SPEED, SPIN_SPEED, ser)
            cv2.putText(frame, "Searching for Tag...", (20, 40), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        cv2.imshow('URC Targeting', frame)
        if cv2.waitKey(1) == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()
    if ser: ser.close()

if __name__ == "__main__":
    urc_task_targeting()