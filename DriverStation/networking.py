import serial
import threading
import time
import queue

SERIAL_PORT = "/dev/ttyACM0"  # Update if needed (Windows = "COM3")
BAUD_RATE = 115200
TIMEOUT = 1.0

class ConnectionManager:
    def __init__(self):
        self.network_thread = threading.Thread(target=self.run)
        self.connected = False

        self.serial_conn = serial.Serial()
        self.serial_conn.port = SERIAL_PORT
        self.serial_conn.baudrate = BAUD_RATE
        self.serial_conn.timeout = TIMEOUT

        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()
        self.running = True

        self.network_thread.daemon = True
        self.network_thread.start()

    def shutdown(self):
        print("Shutting down serial connection...")
        self.running = False
        self.network_thread.join()
        if self.serial_conn.is_open:
            self.serial_conn.close()
        print("Serial connection closed")

    # CRC-16 (MATCHES RP2040 CODE)
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

        # Bit reversal
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
        """Frame payload with header, length, and CRC."""
        msg_len = len(data_bytes)
        header = [0xBE, 0xEF, msg_len & 0xFF, (msg_len >> 8) & 0xFF]
        crc = self.gen_crc16(data_bytes)
        crc_bytes = [crc & 0xFF, (crc >> 8) & 0xFF]
        return bytes(header + list(data_bytes) + crc_bytes)

    def get_next_packet(self):
        """Extract full framed packets from rx_queue."""
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

        total_len = 4 + msg_len + 2  # header + data + CRC
        if self.rx_queue.qsize() < total_len:
            return None  # wait for full packet

        packet = [self.rx_queue.get() for _ in range(total_len)]
        return packet

    def run(self):
        self.running = True
        while self.running:
            if self.connected:
                print("Disconnected")
            self.connected = False
            try:
                self.serial_conn.open()
                self.handle_connection()
                self.serial_conn.close()
            except serial.SerialException as e:
                print(f"Failed to connect: {e}. Sleep 1 second...")
                time.sleep(1)

    def send_packet(self, payload):
        """Send string or bytes payload as framed LoRa packet."""
        if isinstance(payload, str):
            payload = payload.encode()
        packet = self.make_packet(payload)
        self.tx_queue.put(packet)

    def clear_tx_queue(self):
        with self.tx_queue.mutex:
            self.tx_queue.queue.clear()

    def read_data(self):
        try:
            data = self.serial_conn.read(128)
            for byte in data:
                self.rx_queue.put(byte)
            if len(data) > 0:
                self.last_rx_time = time.time()
        except serial.SerialException:
            pass

    def handle_connection(self):
        self.connected = True
        self.last_packet = time.time()
        self.last_rx_time = time.time()
        self.clear_tx_queue()
        print(f"Connected to Radio Feather on {SERIAL_PORT}!")

        while self.running and self.serial_conn.is_open:
            try:
                self.read_data()

                # Timeout if no data received for 1s
                if time.time() - self.last_rx_time > 1.0:
                    print("No packets received from robot for one second. Closing connection.")
                    return

                # TX
                if not self.tx_queue.empty():
                    msg = self.tx_queue.get_nowait()
                    if msg is not None:
                        self.serial_conn.write(msg)
                        self.last_packet = time.time()
                else:
                    time.sleep(0.01)

            except serial.SerialException as e:
                print(f"Serial port error during I/O: {e}")
                return
            except Exception as e:
                print(f"General error during I/O: {e}")
                return

    def is_connected(self):
        return self.connected
