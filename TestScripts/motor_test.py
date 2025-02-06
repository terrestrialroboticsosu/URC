import serial
import sys
import time

BAUD_RATE=115200

if len(sys.argv) < 2:
    print("Invalid number of arguments: please specify serial port.")
    exit(-1)

port=sys.argv[1]
print("Using serial port: ", port)
print("Baud rate: ", BAUD_RATE)

serial_port = serial.Serial(port=port, baudrate=BAUD_RATE)

# Set drive speed to 50%:
# BE EF 81 32 32 32 32 00 00 00 00 00 00
serial_port.write([0xBE, 0xEF, 0x81, 0x32, 0x32, 0x32, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])

time.sleep(5)

# Set deploy to 20%: 
# BE EF 82 20 00 00 00 00 00 00 00 00 00 
serial_port.write([0xBE, 0xEF, 0x82, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00])