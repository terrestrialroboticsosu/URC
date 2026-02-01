import serial
import threading
import time
import queue

SERIAL_PORT = "/dev/ttyACM0"
BAUD_RATE = 115200
TIMEOUT = 0.01

_global_manager = None

def get_connection_manager():
    global _global_manager
    if _global_manager is None:
        _global_manager = ConnectionManager()
    return _global_manager


class ConnectionManager:
    def __init__(self):
        global _global_manager
        if _global_manager is not None and _global_manager is not self:
            return
        _global_manager = self

        # Serial connection
        try:
            self.serial_conn = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=TIMEOUT,
                write_timeout=TIMEOUT
            )
            self.connected = True
            print(f"[ConnectionManager] Connected to {SERIAL_PORT}")
        except serial.SerialException as e:
            print(f"[ConnectionManager] Failed to open {SERIAL_PORT}: {e}")
            self.serial_conn = None
            self.connected = False

        # Queues
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()
        self.running = True
        self.last_send_time = 0  # ← add this

        # Threaded loops
        self.conn_thread = threading.Thread(target=self.connection_loop, daemon=True)
        self.tx_thread = threading.Thread(target=self.tx_loop, daemon=True)
        self.conn_thread.start()
        self.tx_thread.start()

        # Last send time for periodic sending
        self.last_send_time = 0

    def shutdown(self):
        self.running = False
        self.conn_thread.join()
        self.tx_thread.join()
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        print("[ConnectionManager] Serial connection closed")

    def make_packet(self, data_bytes):
        msg_len = len(data_bytes)
        header = [0xBE, 0xEF, msg_len & 0xFF, (msg_len >> 8) & 0xFF]
        return bytes(header + list(data_bytes))

    def send_packet(self, payload):
        """Queue a packet for sending."""
        if isinstance(payload, str):
            payload = payload.encode()
        packet = self.make_packet(payload)
        self.tx_queue.put(packet)
        print(f"[DEBUG] Queued packet: {packet.hex()}")
    def tx_loop(self):
        while self.running:
            now = time.time()
            if now - self.last_send_time >= 1.0:
                self.last_send_time = now

                packet = None  # ← initialize first
                # Discard any old queued packets and keep latest
                try:
                    while True:
                        packet = self.tx_queue.get_nowait()
                except queue.Empty:
                    pass

                if packet is None:
                    # If nothing queued, send heartbeat
                    packet = self.make_packet(b'\x01')

                if self.serial_conn and self.serial_conn.is_open:
                    try:
                        self.serial_conn.write(packet)
                        self.serial_conn.flush()
                        print(f"[TX_LOOP] Sent packet: {packet.hex()}")
                    except serial.SerialException:
                        self.connected = False
            time.sleep(0.01)


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

    def connection_loop(self):
        while self.running:
            if self.serial_conn and not self.serial_conn.is_open:
                try:
                    self.serial_conn.open()
                    self.connected = True
                except serial.SerialException:
                    self.connected = False
                    time.sleep(1)
                    continue
            self.read_data()
            time.sleep(0.005)

    def is_connected(self):
        return self.connected
