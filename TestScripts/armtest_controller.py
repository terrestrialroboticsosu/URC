import pygame
import serial
import time

# ================= SERIAL =================
PORT = "/dev/ttyACM0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

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
last_a = False

# ================= HELPERS =================
def deadzone(v, dz=0.1):
    return 0.0 if abs(v) < dz else v

def scale(v):
    return int((v + 1) * 100)

def clamp(v):
    return max(0, min(200, v))

def send_packet(data):
    """
    Packet:
    [0xBE, 0xEF, left, right, shoulder, elbow, wristB, wristT, turret, stepper, 0, 0]
    """
    packet = bytearray(13)
    packet[0] = 0xBE
    packet[1] = 0xEF

    for i, val in enumerate(data):
        packet[2 + i] = val

    ser.write(packet)

# ================= MAIN LOOP =================
while True:
    pygame.event.pump()

    # ===== MODE TOGGLE =====
    a = js.get_button(0)
    if a and not last_a:
        mode = 1 - mode
        print("MODE:", "ARM" if mode else "DRIVE")
    last_a = a

    # ===== AXES =====
    lx = deadzone(js.get_axis(0))
    ly = -deadzone(js.get_axis(1))
    rx = deadzone(js.get_axis(2))
    ry = -deadzone(js.get_axis(3))

    # ===== DEFAULTS =====
    left = right = 100
    shoulder = elbow = wrist_b = wrist_t = 100
    turret = 100
    stepper = 0

    # ===== MODE LOGIC =====
    if mode == 0:
        # DRIVE
        left  = clamp(scale(ly))
        right = clamp(scale(ry))
    else:
        # ARM
        shoulder = clamp(scale(ry))
        elbow    = clamp(scale(rx))
        wrist_b  = clamp(scale(lx))
        wrist_t  = clamp(scale(ly))

    # ===== TURRET =====
    hat = js.get_hat(0)
    if hat[0] == 1:
        turret = 200
    elif hat[0] == -1:
        turret = 0

    # ===== STEPPER =====
    rb = js.get_button(5)
    lb = js.get_button(4)

    if rb and not lb:
        stepper = 1
    elif lb and not rb:
        stepper = 2

    # ===== SEND =====
    send_packet([
        left,
        right,
        shoulder,
        elbow,
        wrist_b,
        wrist_t,
        turret,
        stepper
    ])

    # ===== DEBUG =====
    print(
        f"{'ARM ' if mode else 'DRIVE'} | "
        f"L:{left:3d} R:{right:3d} | "
        f"S:{shoulder:3d} E:{elbow:3d} | "
        f"WB:{wrist_b:3d} WT:{wrist_t:3d} | "
        f"T:{turret:3d} ST:{stepper}"
    )

    time.sleep(0.05)