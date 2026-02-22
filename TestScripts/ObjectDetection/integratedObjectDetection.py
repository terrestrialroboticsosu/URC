import cv2
import serial
import time
from ultralytics import YOLO

# --- ROBOT CONFIGURATION ---
SERIAL_PORT = "/dev/tty/ACM0"
BAUD_RATE = 115200

CONF_THRESHOLD = 0.856

# Motor protocol:
# 0..200, where 100 = stop, 200 = max forward, 0 = max backward
MOTOR_MIN = 0
MOTOR_MAX = 200
MOTOR_NEUTRAL = 100
MOTOR_RANGE = 100  # distance from neutral to min/max

# Left motor is mounted opposite -> invert its direction
LEFT_MOTOR_INVERTED = True

# Initialize Serial
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except Exception as e:
    print(f"Serial Error: {e}. Running in simulation mode.")
    ser = None


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def speed_to_controller_value(speed_norm: float, inverted: bool) -> int:
    """
    speed_norm: -1.0..+1.0 (negative=reverse, positive=forward)
    inverted: if True, flip direction for that motor
    returns: controller byte 0..200 where 100 is stop
    """
    s = clamp(speed_norm, -1.0, 1.0)
    if inverted:
        s = -s

    val = int(round(MOTOR_NEUTRAL + s * MOTOR_RANGE))
    return clamp(val, MOTOR_MIN, MOTOR_MAX)


def send_robot_command(left_val: int, right_val: int):
    """
    Sends [0xBE, 0xEF, left, right, ...] to motor controller.
    left_val/right_val are controller values in 0..200.
    """
    l_byte = clamp(int(left_val), MOTOR_MIN, MOTOR_MAX)
    r_byte = clamp(int(right_val), MOTOR_MIN, MOTOR_MAX)

    packet = bytearray([0xBE, 0xEF, l_byte, r_byte, 0, 0, 0, 0, 0, 0, 0, 0])

    # Print what is being sent (both decimal and hex)
    print(f"CMD L={l_byte} R={r_byte} | HEX={[hex(b) for b in packet[:4]]}")

    if ser and ser.is_open:
        ser.write(packet)


def drive(forward: float, turn: float):
    """
    forward: -1..+1 (robot forward/back)
    turn:    -1..+1 (positive = turn right)
    """
    f = clamp(forward, -1.0, 1.0)
    t = clamp(turn, -1.0, 1.0)

    # Differential drive mixing:
    # positive turn => speed up LEFT, slow down RIGHT to turn right
    left_speed = clamp(f + t, -1.0, 1.0)
    right_speed = clamp(f - t, -1.0, 1.0)

    left_val = speed_to_controller_value(left_speed, inverted=LEFT_MOTOR_INVERTED)
    right_val = speed_to_controller_value(right_speed, inverted=False)

    print(f"mix forward={f:+.2f} turn={t:+.2f} -> Lspd={left_speed:+.2f} Rspd={right_speed:+.2f}")
    send_robot_command(left_val, right_val)


def main():
    model = YOLO("best_object_detection_URCv2.pt")
    cap = cv2.VideoCapture(0)

    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))

    if not cap.isOpened():
        print("Error: Could not open webcam.")
        return

    print("Targeting: Orange Mallet / 1L Water Bottle")
    print("Strategy: Spin until target found")

    # Tunables
    forward_when_tracking = 0.35   # 35% forward
    turn_gain = 0.55              # how hard to turn based on offset
    spin_turn = 0.45              # in-place spin rate when no target

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model(frame, conf=CONF_THRESHOLD)

        target_found = False

        # Use first detection (highest confidence) if any
        r0 = results[0]
        if len(r0.boxes) > 0:
            box = r0.boxes[0]
            x_center, y_center, w, h = box.xywh[0]

            # offset in [-1, +1], + means target is to the right
            offset = float((x_center - (frame_width / 2)) / (frame_width / 2))

            # turn right if target on right => positive turn
            turn = clamp(offset * turn_gain, -1.0, 1.0)

            drive(forward_when_tracking, turn)
            target_found = True
        else:
            # No target: spin in place (turn without forward)
            drive(0.0, spin_turn)

        annotated_frame = results[0].plot()
        cv2.imshow("Robot Vision", annotated_frame)

        if cv2.waitKey(1) == ord("q"):
            break

    # Stop motors on exit (100 = stop)
    send_robot_command(MOTOR_NEUTRAL, MOTOR_NEUTRAL)
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()