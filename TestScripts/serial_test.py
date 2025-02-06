import serial
import time
import sys

BAUD_RATE=115200
SYNC_BYTE1=0xBE
SYNC_BYTE2=0xEF
DATA_LENGTH=8

MESSAGE_TYPE_HEARBEAT=0x01
MESSAGE_TYPE_INTAKE_POS=0x41
MESSAGE_TYPE_LOG=0x61

def fletcher16(data): 
    sum1 = 0
    sum2 = 0
    for index in range(len(data)):
        sum1 = (sum1 + data[index]) % 255
        sum2 = (sum2 + sum1) % 255
    return bytes([sum1, sum2])

def read_byte(ser):
    return serial_port.read()[0]

def parse_packet(msg_type, data):
    if msg_type == MESSAGE_TYPE_HEARBEAT:
        if(data[0]):
            print("Heartbeat (Enabled=True)")
        else:
            print("Heartbeat (Enabled=False)")
    
    if msg_type == MESSAGE_TYPE_INTAKE_POS:
        angle = (data[0] | (data[1] << 8)) / 100
        print(f"Intake Position: {angle}")

    if msg_type == MESSAGE_TYPE_LOG:
        msg = bytes(data).decode('utf-8')
        print(f"LOG: {msg}")    


 
if len(sys.argv) < 2:
    print("Invalid number of arguments: please specify serial port.")
    exit(-1)

port=sys.argv[1]
print("Using serial port: ", port)
print("Baud rate: ", BAUD_RATE)

serial_port = serial.Serial(port=port, baudrate=BAUD_RATE)

while True:
    rx = read_byte(serial_port)
    if rx != SYNC_BYTE1:
        print(f"Received invalid sync byte! Expected {hex(SYNC_BYTE1)} but got ", hex(rx))
        continue
    
    rx = read_byte(serial_port)
    if rx != SYNC_BYTE2:
        print(f"Received invalid sync byte! Expected {hex(SYNC_BYTE2)} but got ", hex(rx))
        continue

    msg_type = read_byte(serial_port)
    data = list(serial_port.read(DATA_LENGTH))

    packet = [SYNC_BYTE1, SYNC_BYTE2, msg_type] + data
    calc_checksum = fletcher16(packet)
    actual_checksum = serial_port.read(2)
    packet += list(actual_checksum)

    print(f"Packet received: {list(map(hex, packet))}")

    if calc_checksum == actual_checksum:
        parse_packet(msg_type, data)
    else:
        print(f"Checksum did not match! Actual={list(actual_checksum)} Calculated={list(calc_checksum)}")

serial_port.close()
