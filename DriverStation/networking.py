import socket
import threading
import time
import pygame
import struct
import queue

# TCP_IP = '192.168.0.10' 
TCP_IP = '127.0.0.1'
TCP_PORT = 2000
TIMEOUT=1.0

class ConnectionManager: 
    def __init__(self): 
        self.network_thread = threading.Thread(target=self.run)
        self.connected = False
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tx_queue = queue.Queue()
        self.rx_queue = queue.Queue()

        self.network_thread.start()

    def shutdown(self):
        print("Shutting down networking...")
        self.running = False
        self.network_thread.join()
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
            if self.connected:
                print("Disconnected")
            self.connected = False
            try:
                self.socket.setblocking(False)
                self.socket.connect((TCP_IP, TCP_PORT))
                self.handle_connection()
                self.socket.close()
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            except socket.error as e: 
                if e.errno == 106 or "WinError 10022" in str(e): # endpoint already connected
                    print("recreating socket")
                    self.socket.close()
                    self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                elif "WinError 10035" in str(e): # Error 10035 - connecting (but not done yet)
                    print("connecting...")
                    time.sleep(0.25) # Give time to connect
                    self.handle_connection()
                elif e.errno != 115: # Errno 115 = operation in progress
                    self.connected = False
                    print(f"Failed to connect: {e}. Sleep 1 second...")
                    time.sleep(1)

    def send_packet(self, packet):
        self.tx_queue.put(packet)

    def clear_tx_queue(self):
        with self.tx_queue.mutex:
            self.tx_queue.queue.clear()

    def read_data(self):
        try:
            data = self.socket.recv(128)
            for i in range(len(data)):
                self.rx_queue.put(data[i])

            self.last_rx_time = time.time()
        except socket.error:
            pass

    def handle_connection(self):
        self.connected = True
        self.last_packet = time.time()
        self.last_rx_time = time.time()
        self.clear_tx_queue()
        print("connected!") 
        
        while self.running:
            self.read_data()

            if time.time() - self.last_rx_time > 1.0:
                print("No packets received for one second. Killing connection")
                return

            if self.tx_queue.qsize() > 0:
                msg = self.tx_queue.get_nowait()

                if msg is not None:
                    self.socket.send(msg)
                    self.last_packet = time.time()
            else:
                time.sleep(0.1)

    def is_connected(self):
        return self.connected