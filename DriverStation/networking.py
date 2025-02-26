import serial
import threading
import time
import queue

TIMEOUT = 1.0


class ConnectionManager:
    def __init__(self):
        self.network_thread = threading.Thread(target=self.run)
        self.connected = False
        # async serial
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()

        self.network_thread.start()

    def shutdown(self):
        print("Shutting down networking...")
        self.running = False
        self.network_thread.join()
        self.ser.close()
        print("Shutdown network thread")

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
            if not self.connected:
                print("Disconnected")
                self.connected = True
                self.handle_connection()
            time.sleep(0.1)

    def send_packet(self, packet):
        self.tx_queue.put(packet)

    def clear_tx_queue(self):
        with self.tx_queue.mutex:
            self.tx_queue.queue.clear()

    def read_data(self):
        data = self.ser.read(14)
        if data:
            for byte in data:
                self.rx_queue.put(byte)
            self.last_rx_time = time.time()

    def handle_connection(self):
        self.connected = True
        self.last_packet = time.time()
        self.last_rx_time = time.time()
        self.clear_tx_queue()
        print("Connected!")

        while self.running:
            self.read_data()

            if time.time() - self.last_rx_time > TIMEOUT:
                print("No packets received for one second. Killing connection")
                self.connected = False
                return

            if self.tx_queue.qsize() > 0:
                msg = self.tx_queue.get_nowait()

                if msg is not None:
                    self.ser.write(msg)
                    self.last_packet = time.time()
            else:
                time.sleep(0.1)

    def is_connected(self):
        return self.connected
