import serial
import time 
serialport = serial.Serial('COM3', 115200, timeout=1)


i=0
while True:
    for i in range(15):
        serialport.write(str(i).encode())
        serialport.write(b'A'*i)
        serialport.write(b',')
        time.sleep(0.25)
        i += 1
serialport.close()
