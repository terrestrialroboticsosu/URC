import serial
import time

ser = serial.Serial('/dev/ttyACM0', 115200)   # change if needed
time.sleep(2)

packet = bytes([
    0xBE, 0xEF, 0x0B,  # header + motor command
    200, 0x00,         # left stick value
    50,  0x00,         # right stick value
    0,0,0,0,0,0        # padding to reach 13 bytes
])

ser.write(packet)
print("Sent packet:", packet)
