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

    # --- CRC & packet framing ---
    def gen_crc16(self, data):
        out = 0
        bits_read = 0
        byte_idx = 0
        size = len(data)
        while size > 0:
            bit_flag = out >> 15
            out = (out << 1) & 0xFFFF
            out |= (data[byte_idx] >> bits_read) & 1
            bits_read += 1
            if bits_read > 7:
                bits_read = 0
                byte_idx += 1
                size -= 1
            if bit_flag:
                out ^= 0xA001
        for _ in range(16):
            bit_flag = out >> 15
            out = (out << 1) & 0xFFFF
            if bit_flag:
                out ^= 0xA001
        crc = 0
        i = 0x8000
        j = 0x0001
        while i:
            if i & out:
                crc |= j
            i >>= 1
            j <<= 1
        return crc

    def make_packet(self, data_bytes):
        msg_len = len(data_bytes)
        header = [0xBE, 0xEF, msg_len & 0xFF, (msg_len >> 8) & 0xFF]
        crc = self.gen_crc16(data_bytes)
        crc_bytes = [crc & 0xFF, (crc >> 8) & 0xFF]
        return bytes(header + list(data_bytes) + crc_bytes)

    # --- Packet sending ---
    def send_packet(self, payload):
        if isinstance(payload, str):
            payload = payload.encode()
        packet = self.make_packet(payload)
        if self.serial_conn and self.serial_conn.is_open:
            try:
                self.serial_conn.write(packet)
                self.serial_conn.flush()  # send immediately
                print(f"send_packet: sent packet {packet.hex()}")
            except serial.SerialException:
                print("send_packet: failed, serial not connected")
        else:
            print("send_packet: serial not open")


    def tx_loop(self):
        MIN_SEND_INTERVAL = 1.0  # 1 second between packets
        last_send_time = 0

        while self.running:
            try:
                packet = self.tx_queue.get(timeout=0.01)
            except queue.Empty:
                continue

            # Wait until MIN_SEND_INTERVAL has passed since last packet
            now = time.time()
            elapsed = now - last_send_time
            if elapsed < MIN_SEND_INTERVAL:
                wait_time = MIN_SEND_INTERVAL - elapsed
                print(f"tx_loop: waiting {wait_time:.2f}s before sending next packet")
                time.sleep(wait_time)

            if self.serial_conn and self.serial_conn.is_open:
                try:
                    self.serial_conn.write(packet)
                    self.serial_conn.flush()  # force send immediately
                    last_send_time = time.time()
                    # print the packet bytes as hex
                    print(f"tx_loop: sent packet ({len(packet)} bytes): {packet.hex()}")
                except serial.SerialException:
                    print("tx_loop: write failed, requeueing packet")
                    self.tx_queue.put(packet)
                    self.connected = False
                    time.sleep(0.05)
            else:
                self.tx_queue.put(packet)
                time.sleep(0.05)




    # --- Packet receiving ---
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
        if self.rx_queue.qsize() < 6:
            return None
        first = self.rx_queue.queue[0]
        second = self.rx_queue.queue[1]
        if first != 0xBE or second != 0xEF:
            self.rx_queue.get()
            return None
        if self.rx_queue.qsize() < 4:
            return None
        length_L = self.rx_queue.queue[2]
        length_H = self.rx_queue.queue[3]
        msg_len = length_L | (length_H << 8)
        total_len = 4 + msg_len + 2
        if self.rx_queue.qsize() < total_len:
            return None
        packet = [self.rx_queue.get() for _ in range(total_len)]
        return packet

    # --- Connection thread ---
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
