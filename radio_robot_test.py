#!/usr/bin/env python3
import os
import sys
import termios
import time

SYNC1 = 0xBE
SYNC2 = 0xEF
PACKET_LEN = 13

def open_port(path):
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[0] = 0
    attrs[1] = 0
    attrs[2] = attrs[2] | termios.CLOCAL | termios.CREAD
    attrs[3] = 0
    attrs[4] = termios.B115200
    attrs[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    return fd

def read_byte(fd):
    try:
        b = os.read(fd, 1)
        if b:
            return b[0]
    except BlockingIOError:
        return None
    return None

def main():
    if len(sys.argv) < 3:
        print("Usage: python3 serial_forwarder.py <DS_port> <HAL_port>")
        sys.exit(1)

    ds_fd = open_port(sys.argv[1])
    hal_fd = open_port(sys.argv[2])

    saw_first_packet = False
    buffer = []

    while True:
        b = read_byte(ds_fd)
        if b is None:
            continue
            time.sleep(0.001)  # 1ms
        buffer.append(b)

        # Keep looking for a full packet starting with SYNC bytes
        while len(buffer) >= 2:
            if buffer[0] != SYNC1 or buffer[1] != SYNC2:
                buffer.pop(0)
                continue
            if len(buffer) < PACKET_LEN:
                break  # wait for full packet
            packet = buffer[:PACKET_LEN]
            buffer = buffer[PACKET_LEN:]

            if not saw_first_packet:
                saw_first_packet = True
                continue  # discard first packet

            # Forward exactly as-is
            os.write(hal_fd, bytes(packet))
            print("\n=== GOOD DS PACKET RECEIVED ===")
            print(''.join(f"{b:02x}" for b in packet))
            print("================================\n")

if __name__ == "__main__":
    main()
