import socket
import threading
import time
import queue
import serial
import serial.threaded

TCP_IP = '192.168.0.10' 
# TCP_IP = '127.0.0.1'
TCP_PORT = 2000
TIMEOUT=1.0

class ConnectionManager: 

    def __init__(self, port, baud_rate):
        self.port = port
        self.baud_rate = baud_rate
        self.rx_buf = queue.SimpleQueue

        try:
            self.serial = serial.Serial(port=port, baudrate=baud_rate)
        except Exception as e:
            self.serial = None
            print("Failed to open serial port: {}", e)

    def shutdown(self):
        print("Shutting down networking...")
        self.running = False
        self.network_thread.join()
        print("Shutdown network thread")

    def get_next_packet(self): 

        body = []
        if(self.serial != None):
            return body
        
        packet_length = 13
        while len(self.rx_buf) > packet_length:
            syncByte1 = self.rx_buf.get()
            syncByte2 = self.rx_buf.get()
            message_type = None
            if syncByte1 == 0xBE and syncByte2 == 0xEF:
                message_type = self.rx_buf.get()
            
                for i in range(packet_length):
                    body.append(self.rx_buf.put())

                #remove checksum
                self.rx_buf.empty()
                print("{} sent {} packet", self.port, message_type)

                return body
            else:
                print("{} send incorrect sync byte {}", self.port, syncByte2)
        
        return body
                

    def process_serial(self):
        if self.serial.in_waiting > 0:
            bytes_read = self.serial.read_all()
            if bytes_read != None:
                for byte in bytes_read:
                    self.rx_buf.put(byte)
            
            
    def read_heart_beat(self):
        return True if (self.rx_buf.put() == 0xBE and self.rx_buf.put == 0xEF) else False


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