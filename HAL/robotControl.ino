import pygame
import serial
import time

# ================= SERIAL =================
PORT = "/dev/ttyACM0"   # change to your port (Windows: "COM3", Mac: "/dev/tty.usbmodem...")
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)  # wait for Arduino to reset after serial open

# ================= CONTROLLER =================
pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No controller found!")
    exit()

js = pygame.joystick.Joystick(0)
js.init()
print("Controller:", js.get_name())

# ================= STATE =================
mode = 0        # 0 = DRIVE, 1 = ARM
last_a = False  # for A button edge detection

# ================= HELPERS =================
def deadzone(v, dz=0.1):
    """Ignore small stick movements near center."""
    return 0.0 if abs(v) < dz else v

def scale(v):
    """Map -1..1 -> 0..200 (100 = neutral/stop)."""
    return int((v + 1) * 100)

def clamp(v):
    return max(0, min(200, v))

def send_packet(data):
    """
    Build and send a 13-byte packet:
      [0xBE, 0xEF, mode, left, right, shoulder, elbow, wristB, wristT, turret, stepper, 0, 0]
    Speed limiting is handled entirely on the Arduino side.
    """
    packet = bytearray(13)
    packet[0] = 0xBE
    packet[1] = 0xEF
    for i, val in enumerate(data):
        if 2 + i < 13:
            packet[2 + i] = val
    ser.write(packet)

# ================= MAIN LOOP =================
while True:
    pygame.event.pump()

    # ===== Mode toggle - A button (button 0) =====
    a = js.get_button(0)
    if a and not last_a:
        mode = 1 - mode
        print("MODE:", "ARM" if mode else "DRIVE")
    last_a = a

    # ===== Read axes with deadzone =====
    lx = deadzone(js.get_axis(0))    # Left stick X
    ly = -deadzone(js.get_axis(1))   # Left stick Y  (inverted: up = positive)
    rx = deadzone(js.get_axis(3))    # Right stick X
    ry = -deadzone(js.get_axis(4))   # Right stick Y (inverted: up = positive)

    # ===== Defaults - all neutral =====
    left = right = shoulder = elbow = wrist_b = wrist_t = turret = 100
    stepper = 0

    if mode == 0:
        # ----- DRIVE MODE (tank) -----
        # Left stick Y  -> left wheels
        # Right stick Y -> right wheels
        left  = clamp(scale(ly))
        right = clamp(scale(ry))

    else:
        # ----- ARM MODE -----
        # Right stick Y -> shoulder (up/down)
        # Right stick X -> elbow (in/out)
        # Left  stick X -> wrist bend
        # Left  stick Y -> wrist twist
        shoulder = clamp(scale(ry))
        elbow    = clamp(scale(rx))
        wrist_b  = clamp(scale(lx))
        wrist_t  = clamp(scale(ly))

    # ===== TURRET - D-pad left/right (active in both modes) =====
    hat = js.get_hat(0)  # (x, y): x = -1 left, 0 neutral, 1 right
    if hat[0] == 1:
        turret = 200     # rotate right
    elif hat[0] == -1:
        turret = 0       # rotate left
    else:
        turret = 100     # neutral

    # ===== STEPPER - bumpers open/close end effector (active in both modes) =====
    rb = js.get_button(5)  # right bumper -> open
    lb = js.get_button(4)  # left bumper  -> close
    if rb and not lb:
        stepper = 1        # open
    elif lb and not rb:
        stepper = 2        # close
    else:
        stepper = 0        # stop

    # ===== SEND =====
    send_packet([
        mode,       # packet[2]  - current mode (informational)
        left,       # packet[3]  - left drive
        right,      # packet[4]  - right drive
        shoulder,   # packet[5]  - shoulder
        elbow,      # packet[6]  - elbow
        wrist_b,    # packet[7]  - wrist bend
        wrist_t,    # packet[8]  - wrist twist
        turret,     # packet[9]  - turret
        stepper,    # packet[10] - stepper (0=stop, 1=open, 2=close)
    ])

    # ===== DEBUG PRINT =====
    print(
        f"MODE: {'ARM ' if mode else 'DRIVE'} | "
        f"L:{left:3d} R:{right:3d} | "
        f"Shldr:{shoulder:3d} Elbow:{elbow:3d} | "
        f"WristB:{wrist_b:3d} WristT:{wrist_t:3d} | "
        f"Turret:{turret:3d} Stepper:{stepper}"
    )

    time.sleep(0.05)  # ~20 Hz update rate
