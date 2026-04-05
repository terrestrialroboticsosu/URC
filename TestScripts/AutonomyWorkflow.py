import cv2
import cv2.aruco as aruco
import numpy as np
import os
import time
import serial
import math

# ====================== CONFIGURATION ======================
# --- Autonomy Parameters ---
GPS_ARRIVAL_THRESHOLD = 2.0  # Meters. If within this distance, start visual search
EST_DRIVE_SPEED_MPS = 0.5    # Estimated m/s when driving at DRIVE_SPEED
EST_TURN_SPEED_DPS = 45.0    # Estimated deg/s when turning at SPIN_SPEED

# --- Motor & Driving Parameters (From your tests) ---
DESIRED_DISTANCE = 1.8    
KP_FORWARD = 45           
KP_TURN = 35              
TURN_MULTIPLIER = 1.8     
MIN_DRIVE_POWER = 35      
MAX_SPEED = 100           
LEFT_MOTOR_BIAS = 10      # Multiplier to fix weak left motor
SPIN_SPEED = 50           # Base speed for Panorama/Turning
DRIVE_SPEED = 60          # Base speed for GPS/Square driving

# --- Camera & Hardware Parameters ---
TAG_SIZE = 0.145          
TARGET_COLOR = (0, 255, 255)
CALIB_FILENAME = "webcam_calibration.npz"
CAMERA_INDEX = 33         

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200

# ====================== HELPERS ======================

class RoverState:
    NAVIGATING_GPS = "NAV_GPS"
    PANORAMA = "PANORAMA"
    EXPANDING_SQUARE = "EXPANDING_SQUARE"
    VISUAL_SERVO = "VISUAL_SERVO"
    SUCCESS = "SUCCESS"

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

class NavigationMath:
    @staticmethod
    def get_distance_and_bearing(lat1, lon1, lat2, lon2):
        R = 6371000  # Radius of Earth in meters
        phi_1 = math.radians(lat1)
        phi_2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)

        a = math.sin(delta_phi / 2.0)**2 + math.cos(phi_1) * math.cos(phi_2) * math.sin(delta_lambda / 2.0)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c

        y = math.sin(delta_lambda) * math.cos(phi_2)
        x = math.cos(phi_1) * math.sin(phi_2) - math.sin(phi_1) * math.cos(phi_2) * math.cos(delta_lambda)
        bearing = (math.degrees(math.atan2(y, x)) + 360) % 360

        return distance, bearing

    @staticmethod
    def calculate_heading_error(current_heading, target_bearing):
        error = target_bearing - current_heading
        if error > 180: error -= 360
        if error < -180: error += 360
        return error

class ArduinoBridge:
    def __init__(self, port=SERIAL_PORT, baudrate=BAUD_RATE):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(2)
            print(f"Connected to Arduino on {port}")
        except Exception as e:
            self.ser = None
            print(f"WARNING: Arduino error ({e}). Running simulation mode.")

    def send_motor_command(self, left, right):
        if not self.ser: return
        
        # Match your exact inversion logic
        left_inv = -int(left)
        right_inv = -int(right)

        left_conv  = int(clamp(left_inv + 100, 0, 200))
        right_conv = int(clamp(right_inv + 100, 0, 200))

        # 13-Byte Packet (Must match PACKET_SIZE 13 in Arduino)
        packet = [0xBE, 0xEF, left_conv, right_conv, 100, 100, 100, 100, 100, 100, 0, 0, 0]
        self.ser.write(bytearray(packet))

# ====================== AUTONOMY BRAIN ======================

class FullAutonomyWorkflow:
    def __init__(self):
        self.state = RoverState.NAVIGATING_GPS
        self.target_lat = 0.0
        self.target_lon = 0.0
        
        # Search variables
        self.total_rotated = 0.0
        self.spiral_step = 2.0
        self.spiral_iteration = 0
        self.leg_dist = 0.0
        self.turn_angle = 0.0
        self.spiral_is_driving = True

    def set_target_waypoint(self, lat, lon):
        self.target_lat = lat
        self.target_lon = lon
        self.state = RoverState.NAVIGATING_GPS
        print(f"\n--- NEW MISSION: Navigating to {lat}, {lon} ---")

    def apply_motor_physics(self, left_motor, right_motor):
        """Applies your custom friction floors and bias to any generic motor command."""
        # 1. Left Wheel Bias
        left_motor = left_motor * LEFT_MOTOR_BIAS

        # 2. Overcome Friction Floor
        if abs(left_motor) > 1:
            left_motor = np.sign(left_motor) * max(abs(left_motor), MIN_DRIVE_POWER)
        if abs(right_motor) > 1:
            right_motor = np.sign(right_motor) * max(abs(right_motor), MIN_DRIVE_POWER)

        # 3. Final Clamp
        left_motor = clamp(left_motor, -MAX_SPEED, MAX_SPEED)
        right_motor = clamp(right_motor, -MAX_SPEED, MAX_SPEED)
        
        return left_motor, right_motor

    def update(self, dt, current_lat, current_lon, current_heading, z_dist, centering_error):
        left_motor = 0.0
        right_motor = 0.0

        # --- PRIORITY OVERRIDE: CAMERA LOCK ---
        if z_dist is not None and self.state != RoverState.SUCCESS:
            self.state = RoverState.VISUAL_SERVO

        # --- STATE MACHINE LOGIC ---
        if self.state == RoverState.NAVIGATING_GPS:
            dist_to_gps, bearing = NavigationMath.get_distance_and_bearing(
                current_lat, current_lon, self.target_lat, self.target_lon
            )
            heading_error = NavigationMath.calculate_heading_error(current_heading, bearing)

            if dist_to_gps <= GPS_ARRIVAL_THRESHOLD:
                print(f"[{self.state}] GPS Area Reached! Initiating Panorama Search.")
                self.state = RoverState.PANORAMA
                self.total_rotated = 0.0
            else:
                # Turn to face waypoint
                if abs(heading_error) > 15.0:
                    sign = 1 if heading_error > 0 else -1
                    left_motor = SPIN_SPEED * sign
                    right_motor = -SPIN_SPEED * sign
                else:
                    # Drive forward
                    left_motor = DRIVE_SPEED
                    right_motor = DRIVE_SPEED

        elif self.state == RoverState.PANORAMA:
            if self.total_rotated < 360.0:
                left_motor = -SPIN_SPEED
                right_motor = SPIN_SPEED
                self.total_rotated += (EST_TURN_SPEED_DPS * dt)
            else:
                print(f"[{self.state}] Panorama complete. Starting Expanding Square.")
                self.state = RoverState.EXPANDING_SQUARE
                self.spiral_iteration = 0
                self.leg_dist = 0.0
                self.spiral_is_driving = True

        elif self.state == RoverState.EXPANDING_SQUARE:
            target_leg_length = (math.floor(self.spiral_iteration / 2) + 1) * self.spiral_step

            if self.spiral_is_driving:
                if self.leg_dist < target_leg_length:
                    left_motor = DRIVE_SPEED
                    right_motor = DRIVE_SPEED
                    self.leg_dist += (EST_DRIVE_SPEED_MPS * dt)
                else:
                    self.spiral_is_driving = False
                    self.turn_angle = 0.0
            else:
                if self.turn_angle < 90.0:
                    left_motor = SPIN_SPEED
                    right_motor = -SPIN_SPEED
                    self.turn_angle += (EST_TURN_SPEED_DPS * dt)
                else:
                    self.spiral_is_driving = True
                    self.leg_dist = 0.0
                    self.spiral_iteration += 1

        elif self.state == RoverState.VISUAL_SERVO:
            if z_dist <= DESIRED_DISTANCE:
                print(f"!!! TARGET REACHED: {z_dist:.2f}m !!!")
                self.state = RoverState.SUCCESS
            else:
                # Your exact mathematical servoing logic
                forward_output = (z_dist - DESIRED_DISTANCE) * KP_FORWARD
                turn_output = (centering_error * KP_TURN) * TURN_MULTIPLIER

                left_motor = forward_output + turn_output
                right_motor = forward_output - turn_output

        # --- APPLY PHYSICAL MOTOR FIXES ---
        if self.state not in [RoverState.SUCCESS]:
            left_motor, right_motor = self.apply_motor_physics(left_motor, right_motor)
        else:
            left_motor, right_motor = 0, 0 # Dead stop

        return left_motor, right_motor

# ====================== MAIN WORKFLOW ======================

def run_mission():
    camera_matrix, dist_coeffs = load_calibration(CALIB_FILENAME)
    if camera_matrix is None: return

    arduino = ArduinoBridge()
    brain = FullAutonomyWorkflow()
    
    # 1. Set the GPS Mission coordinates
    brain.set_target_waypoint(lat=38.4065, lon=-110.7919)

    # 2. Camera Setup
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(aruco_dict, aruco.DetectorParameters())
    
    cap = cv2.VideoCapture(CAMERA_INDEX)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    last_time = time.time()
    
    # For drawing a persistent box if we lose sight temporarily
    frame_center_x = cap.get(cv2.CAP_PROP_FRAME_WIDTH) / 2

    while True:
        # Calculate dt for the search state machine
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        ret, frame = cap.read()
        if not ret: break

        # -------------------------------------------------------------
        # INSERT SENSOR DATA HERE
        # Update these variables with your actual GPS/Compass feeds
        mock_current_lat = 38.4060  
        mock_current_lon = -110.7919
        mock_current_heading = 0.0  
        # -------------------------------------------------------------

        z_dist = None
        centering_error = None

        # ArUco Detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = detector.detectMarkers(gray)

        if ids is not None and len(ids) > 0:
            obj_pts = np.array([[-TAG_SIZE/2, -TAG_SIZE/2, 0],[TAG_SIZE/2, -TAG_SIZE/2, 0],
                                [TAG_SIZE/2, TAG_SIZE/2, 0],[-TAG_SIZE/2, TAG_SIZE/2, 0]], dtype=np.float32)
            img_pts = corners[0].reshape((4,2)).astype(np.float32)
            success, _, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)

            if success:
                z_dist = tvec.flatten()[2]
                tag_center_x = np.mean(img_pts[:, 0])
                centering_error = (tag_center_x - frame_center_x) / frame_center_x
                
                # Visuals
                cv2.circle(frame, (int(tag_center_x), int(np.mean(img_pts[:,1]))), 10, TARGET_COLOR, -1)

        # Brain processes data and returns motor speeds
        l_motor, r_motor = brain.update(
            dt, mock_current_lat, mock_current_lon, mock_current_heading, 
            z_dist, centering_error
        )

        # Send to Arduino
        arduino.send_motor_command(l_motor, r_motor)

        # If Mission Complete, safely shutdown
        if brain.state == RoverState.SUCCESS:
            for _ in range(5): # Spam stop to be safe
                arduino.send_motor_command(0, 0)
                time.sleep(0.05)
            print("Mission Completed Successfully.")
            break

        # UI Overlay
        ui_text = f"[{brain.state}] L:{int(l_motor)} R:{int(r_motor)}"
        if z_dist:
            ui_text += f" DIST:{z_dist:.2f}m"
            
        color = TARGET_COLOR if brain.state == RoverState.VISUAL_SERVO else (0, 0, 255)
        cv2.putText(frame, ui_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        cv2.imshow('URC Autonomy', frame)
        
        if cv2.waitKey(1) == ord('q'): break

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    if arduino.ser: 
        arduino.send_motor_command(0, 0)
        arduino.ser.close()

if __name__ == "__main__":
    run_mission()