# networking.py
import serial
import threading
import time
import queue

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 0.01

class ConnectionManager:
    def __init__(self):
        # Open serial immediately
        try:
            self.serial_conn = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=TIMEOUT,
                write_timeout=TIMEOUT
            )
            self.connected = True
            print(f"Connected to {SERIAL_PORT}!")
        except serial.SerialException as e:
            print(f"Failed to open {SERIAL_PORT}: {e}")
            self.serial_conn = None
            self.connected = False

        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()
        self.running = True

        # Start threads
        self.conn_thread = threading.Thread(target=self.connection_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
        self.conn_thread.start()
        self.tx_thread.start()

    def shutdown(self):
        self.running = False
        self.conn_thread.join()
        self.tx_thread.join()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        print("Serial connection closed")

    # Packet framing (no CRC)
    def make_packet(self, data_bytes):
        msg_len = len(data_bytes)
        header = [0xBE, 0xEF, msg_len & 0xFF, (msg_len >> 8) & 0xFF]
        return bytes(header + list(data_bytes))

    # Packet sending
    # FIXED: no longer sends immediately, only queues packet
    def send_packet(self, payload):
        if isinstance(payload, str):
            payload = payload.encode()
        packet = self.make_packet(payload)
        self.tx_queue.put(packet)
        
    def tx_loop(self):
        while self.running:
            try:
                packet = self.tx_queue.get(timeout=0.01)
            except queue.Empty:
                continue

            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(packet)
                    self.serial_conn.flush()
                    print(f"tx_loop: sent packet ({len(packet)} bytes): {packet.hex()}")
                except serial.SerialException:
                    # DON'T re-queue, just drop and reconnect
                    self.connected = False
                    time.sleep(0.05)
            else:
                # DON'T re-queue here either
                time.sleep(0.05)
        
    # Packet receiving
    def read_data(self):
        if not self.serial_conn or not self.serial_conn.is_open:
            return
        try:
            data = self.serial_conn.read(128)
            for byte in data:
                self.rx_queue.put(byte)
        except serial.SerialException:
            self.connected = False

    def get_next_packet(self):
        if self.rx_queue.qsize() < 4:
            return None

        first = self.rx_queue.queue[0]
        second = self.rx_queue.queue[1]

        if first != 0xBE or second != 0xEF:
            self.rx_queue.get()
            return None

        length_L = self.rx_queue.queue[2]
        length_H = self.rx_queue.queue[3]
        msg_len = length_L | (length_H << 8)
        total_len = 4 + msg_len

        if self.rx_queue.qsize() < total_len:
            return None

        packet = [self.rx_queue.get() for _ in range(total_len)]
        return packet

    # Connection thread
    def connection_loop(self):
        while self.running:
            if self.serial_conn and not self.serial_conn.is_open:
                try:
                    self.serial_conn.open()
                    self.connected = True
                    print(f"Connected to {SERIAL_PORT}!")
                except serial.SerialException:
                    self.connected = False
                    time.sleep(1)
                    continue
            self.read_data()
            time.sleep(0.005)

    def is_connected(self):
        return self.connected
