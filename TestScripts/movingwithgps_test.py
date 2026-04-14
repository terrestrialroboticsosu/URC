import numpy as np
import time
import serial
import math

# ====================== CONFIGURATION ======================

DESIRED_DISTANCE = 1.8    
KP_FORWARD = 45           
KP_TURN = 35              
TURN_MULTIPLIER = 1.8     
MIN_DRIVE_POWER = 35      
MAX_SPEED = 100           
earthRadius = 6371000
SERIAL_PORT_GPS = "/dev/cu.usbmodem101"   # GPS serial port (change as needed)
SERIAL_PORT_MOTORS = "/dev/ttyACM0"
BAUD_RATE = 115200
GPS_BAUD = 9600                    # Most GPS modules use 9600

# ====================== MATH HELPERS ======================

def nmea_to_decimal(raw, direction):
    """
    Converts raw NMEA coordinate (DDDMM.MMMM) + direction letter to signed decimal degrees.
    e.g. "4007.3821", "N" -> 40.12303...
    """
    if not raw or not direction:
        return None
    dot = raw.index('.')
    deg = float(raw[:dot - 2])
    mins = float(raw[dot - 2:])
    decimal = deg + mins / 60.0
    if direction in ('S', 'W'):
        decimal *= -1
    return decimal

def parse_gprmc(sentence):
    """
    Parses a $GPRMC NMEA sentence.
    Returns (lat, lon) in decimal degrees, or (None, None) if invalid/no fix.
    """
    try:
        parts = sentence.strip().split(',')
        if not parts[0].endswith('RMC'):
            return None, None
        status = parts[2]        # 'A' = active/valid, 'V' = void/no fix
        if status != 'A':
            return None, None    # No GPS fix yet
        lat = nmea_to_decimal(parts[3], parts[4])
        lon = nmea_to_decimal(parts[5], parts[6])
        return lat, lon
    except Exception:
        return None, None

def get_gps_fix(gps_ser):
    """
    Reads lines from the GPS serial port until a valid GPRMC fix is found.
    Blocks until a fix is acquired.
    """
    print("Waiting for GPS fix...")
    while True:
        try:
            line = gps_ser.readline().decode('ascii', errors='replace').strip()
            if '$GPRMC' in line or '$GNRMC' in line:
                lat, lon = parse_gprmc(line)
                if lat is not None and lon is not None:
                    print(f"  GPS Fix acquired: lat={lat:.6f}, lon={lon:.6f}")  # <-- added
                    return lat, lon
                else:
                    print(f"  No fix yet: {line}")
        except Exception as e:
            print(f"  GPS read error: {e}")

# ====================== NAVIGATION MATH ======================

def calculateDistance(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    theta = 2 * math.asin(math.sqrt(
        math.sin((lat2 - lat1) / 2) ** 2 +
        math.cos(lat1) * math.cos(lat2) * math.sin((lon2 - lon1) / 2) ** 2
    ))
    return earthRadius * theta

def calculateHeading(lat1, lon1, lat2, lon2):
    lat1 = math.radians(lat1)
    lat2 = math.radians(lat2)
    lon1 = math.radians(lon1)
    lon2 = math.radians(lon2)
    deltaLon = lon2 - lon1
    xC = math.sin(deltaLon) * math.cos(lat2)
    yC = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(deltaLon)
    beta = math.atan2(xC, yC)
    return (math.degrees(beta) + 360) % 360

# ====================== HELPERS ======================

def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)

def send_motor_command(left, right, ser):
    left_inv  = -int(left)
    right_inv = -int(right)
    left_conv  = int(clamp(left_inv  + 100, 0, 200))
    right_conv = int(clamp(right_inv + 100, 0, 200))
    packet = [0xBE, 0xEF, left_conv, right_conv, 100, 100, 100, 100, 100, 100, 0, 0]
    ser.write(bytearray(packet))

def heading_error(current_heading, target_heading):
    """
    Returns the shortest signed angular difference in degrees.
    Positive = need to turn right, Negative = need to turn left.
    """
    error = (target_heading - current_heading + 360) % 360
    if error > 180:
        error -= 360
    return error

# ====================== MAIN ======================

def gps_navigation_task():
    # --- Open GPS serial port ---
    try:
        gps_ser = serial.Serial(SERIAL_PORT_GPS, GPS_BAUD, timeout=1)
        time.sleep(2)
        print("GPS serial connected")
    except Exception as e:
        print(f"GPS serial error: {e}")
        return

    # --- Open motor serial port ---
    try:
        motor_ser = serial.Serial(SERIAL_PORT_MOTORS, BAUD_RATE, timeout=1)
        time.sleep(2)
        print("Motor serial connected")
    except Exception as e:
        print(f"Motor serial error: {e}")
        motor_ser = None
        
     # --- Get starting position first ---
    start_lat, start_lon = get_gps_fix(gps_ser)
    print(f"Start position: {start_lat:.6f}, {start_lon:.6f}")

    # --- Flush GPS buffer, THEN ask for input ---
    gps_ser.reset_input_buffer()   # clears any queued GPS data
    time.sleep(0.5)                # brief pause to let buffer settle

    print("\nEnter destination coordinates:")
    dest_lat = float(input("Destination latitude (decimal degrees): "))
    dest_lon = float(input("Destination longitude (decimal degrees): "))

    # --- Get starting position ---
    start_lat, start_lon = get_gps_fix(gps_ser)
    print(f"Start position: {start_lat:.6f}, {start_lon:.6f}")

    # --- Enter destination ---
    print("\nEnter destination coordinates:")
    dest_lat = float(input("Destination latitude (decimal degrees): "))
    dest_lon = float(input("Destination longitude (decimal degrees): "))

    target_heading = calculateHeading(start_lat, start_lon, dest_lat, dest_lon)
    total_distance = calculateDistance(start_lat, start_lon, dest_lat, dest_lon)
    print(f"\nTarget heading: {target_heading:.1f}°")
    print(f"Total distance: {total_distance:.2f} m")

    # ====================== PHASE 1: ROTATE TO HEADING ======================
    # NOTE: This requires your robot to report its current compass/IMU heading.
    # Replace `get_robot_heading()` with however you read your compass/IMU.
    # If you don't have one, you can skip rotation and rely purely on GPS correction.

    print("\n[Phase 1] Rotating to target heading...")
    HEADING_TOLERANCE = 5.0   # degrees — acceptable heading error before driving

    while True:
        current_heading = get_robot_heading()   # <-- plug in your IMU/compass read here
        error = heading_error(current_heading, target_heading)

        print(f"  Current: {current_heading:.1f}°  Target: {target_heading:.1f}°  Error: {error:.1f}°")

        if abs(error) <= HEADING_TOLERANCE:
            print("  Heading locked. Moving to drive phase.")
            if motor_ser:
                send_motor_command(0, 0, motor_ser)
            break

        # Rotate in place: one motor forward, one back
        turn_power = clamp(KP_TURN * (error / 180.0) * TURN_MULTIPLIER, -MAX_SPEED, MAX_SPEED)
        turn_power = math.copysign(max(abs(turn_power), MIN_DRIVE_POWER), turn_power)

        if motor_ser:
            # Positive error = turn right: left forward, right back
            send_motor_command(turn_power, -turn_power, motor_ser)
        time.sleep(0.05)

    # ====================== PHASE 2: DRIVE TO DESTINATION ======================

    print("\n[Phase 2] Driving to destination...")

    while True:
        # --- Read current GPS position ---
        cur_lat, cur_lon = get_gps_fix(gps_ser)

        # --- Recalculate distance and heading error ---
        dist = calculateDistance(cur_lat, cur_lon, dest_lat, dest_lon)
        target_hdg = calculateHeading(cur_lat, cur_lon, dest_lat, dest_lon)
        current_heading = get_robot_heading()  # <-- same IMU/compass read
        h_error = heading_error(current_heading, target_hdg)

        print(f"  Distance: {dist:.2f} m | Heading error: {h_error:.1f}°")

        # --- Stop if close enough ---
        if dist <= DESIRED_DISTANCE:
            print("Destination reached!")
            break

        # --- Forward power (proportional to distance, clamped) ---
        forward_output = clamp(KP_FORWARD * (dist / total_distance), MIN_DRIVE_POWER, MAX_SPEED)

        # --- Turn correction ---
        turn_output = KP_TURN * (h_error / 180.0) * TURN_MULTIPLIER

        # --- Differential mix ---
        left_motor  = forward_output + turn_output
        right_motor = forward_output - turn_output

        # --- Friction floor ---
        if abs(left_motor) > 1:
            left_motor  = math.copysign(max(abs(left_motor),  MIN_DRIVE_POWER), left_motor)
        if abs(right_motor) > 1:
            right_motor = math.copysign(max(abs(right_motor), MIN_DRIVE_POWER), right_motor)

        # --- Final clamp ---
        left_motor  = clamp(left_motor,  -MAX_SPEED, MAX_SPEED)
        right_motor = clamp(right_motor, -MAX_SPEED, MAX_SPEED)

        if motor_ser:
            send_motor_command(left_motor, right_motor, motor_ser)

        time.sleep(0.1)

    # --- Stop motors ---
    if motor_ser:
        for _ in range(5):
            send_motor_command(0, 0, motor_ser)
            time.sleep(0.05)
        motor_ser.close()
    gps_ser.close()


if __name__ == "__main__":
    gps_navigation_task()