import serial
import time
from pynput import keyboard

# ================= SERIAL =================
PORT = "/dev/ttyACM0"   # Linux / Mac
# PORT = "COM6"        # Windows
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=0.1)
time.sleep(2)

print("Keyboard arm + independent drive control active")
print("Hold key to move, release to stop")

# ================= KEY MAP =================
keymap = {
    # Shoulder
    'w': ('w', 'W'),
    's': ('s', 'W'),

    # Elbow
    'e': ('e', 'E'),
    'd': ('d', 'E'),

    # Wrist
    't': ('t', 'T'),
    'g': ('g', 'T'),
    'y': ('y', 'T'),
    'h': ('h', 'T'),

    # Drive LEFT
    'u': ('u', 'U'),
    'j': ('j', 'J'),

    # Drive RIGHT
    'i': ('i', 'I'),
    'k': ('k', 'K'),

    # Stepper
    'o': ('o', 'O'),
    'l': ('l', 'O'),

    # Turret
    'q': ('q', 'Q'),
    'a': ('a', 'A'),
}

pressed = set()

def send(c):
    ser.write(c.encode())

def on_press(key):
    try:
        k = key.char
        if k in keymap and k not in pressed:
            pressed.add(k)
            send(keymap[k][0])
    except AttributeError:
        pass

def on_release(key):
    try:
        k = key.char
        if k in pressed:
            pressed.remove(k)
            send(keymap[k][1])
    except AttributeError:
        pass

with keyboard.Listener(
    on_press=on_press,
    on_release=on_release,
    suppress=True
) as listener:
    listener.join()
