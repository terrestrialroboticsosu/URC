import serial
import time
import struct

ser = serial.Serial("/dev/ttyACM0", 115200)

START_BYTE1 = 0xBE
START_BYTE2 = 0xEF
PACKET_TYPE_HEARTBEAT = 0x01

def crc16(data):
    crc = 0x0000
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

while True:
    payload = struct.pack("!bb", 1, 1)  # robot_enabled=1, rp2040_connected=1
    packet_len = len(payload) + 1  # + message_type
    packet = struct.pack("!BBH", START_BYTE1, START_BYTE2, packet_len)
    packet += struct.pack("!B", PACKET_TYPE_HEARTBEAT) + payload
    c = crc16(packet[4:])  # calculate CRC on messageType + data
    packet += struct.pack("<H", c)
    ser.write(packet)
    time.sleep(0.1)
