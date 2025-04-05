import serial
import threading
import time
import queue

SERIAL_PORT = "/dev/ttyACM0" 
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

        self.network_thread.start()

    def shutdown(self):
        print("Shutting down serial connection...")
        self.running = False
        self.network_thread.join()
        if self.serial_conn.is_open:
            self.serial_conn.close()
        print("Serial connection closed")

    def get_next_packet(self):
        if self.rx_queue.qsize() < 11:
            return None

        packet = []
        for _ in range(11):
            packet.append(self.rx_queue.get())
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

    def send_packet(self, packet):
        self.tx_queue.put(packet)

    def clear_tx_queue(self):
        with self.tx_queue.mutex:
            self.tx_queue.queue.clear()

    def read_data(self):
        try:
            data = self.serial_conn.read(128)
            for byte in data:
                self.rx_queue.put(byte)
            self.last_rx_time = time.time()
        except serial.SerialException:
            pass

    def handle_connection(self):
        self.connected = True
        self.last_packet = time.time()
        self.last_rx_time = time.time()
        self.clear_tx_queue()
        print("Connected to serial device!")

        while self.running and self.serial_conn.is_open:
            self.read_data()

            if time.time() - self.last_rx_time > 1.0:
                print("No packets received for one second. Closing connection.")
                return

            if not self.tx_queue.empty():
                msg = self.tx_queue.get_nowait()
                if msg is not None:
                    self.serial_conn.write(msg)
                    self.last_packet = time.time()
            else:
                time.sleep(0.1)

    def is_connected(self):
        return self.connected
