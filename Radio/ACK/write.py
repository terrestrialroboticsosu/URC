import board
import digitalio
import adafruit_rfm9x
import time

# Define radio frequency in MHz. Must match your module.
RADIO_FREQ_MHZ = 915.0

# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)

# Initialize RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)

# Read the image as bytes
with open("image.jpg", "rb") as img_file:
    image_data = img_file.read()

# Chunk size (LoRa limit: ~250 bytes, using 200 for safety)
CHUNK_SIZE = 200
num_packets = (len(image_data) // CHUNK_SIZE) + 1
ack_timeout = 2  # Timeout for acknowledgment

print(f"Total packets to send: {num_packets}")

sequence_number = 0
while sequence_number < num_packets:
    start_byte = sequence_number * CHUNK_SIZE
    end_byte = start_byte + CHUNK_SIZE
    packet_data = image_data[start_byte:end_byte]

    # Add sequence number at the start (e.g., "0001:<data>")
    packet_message = f"{sequence_number:04d}:".encode("ascii") + packet_data
    print(f"Sending packet {sequence_number}")

    # Send the packet
    rfm95.send(packet_message)


    # Wait for ACK
    start_time = time.monotonic()
    ack_received = False

    while time.monotonic() - start_time < ack_timeout:
        packet = rfm95.receive()
        if packet is not None:
            ack = packet.decode("ascii").strip()
            if ack == f"ACK{sequence_number:04d}":
                print(f"ACK received for packet {sequence_number}")
                ack_received = True
                break

    if not ack_received:
        print(f"Timeout! Resending packet {sequence_number}")
    else:
        sequence_number += 1
        print("Ack Recieved")
