import serial
import time

SERIAL_PORT = "/dev/ttyACM0"  
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

try:
    while True:
        simulated_data = b'\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0A\x0B'
        ser.write(simulated_data) 
        print(f"Sent: {simulated_data}")
        time.sleep(1)

except KeyboardInterrupt:
    print("Stopping simulation...")
    ser.close()
