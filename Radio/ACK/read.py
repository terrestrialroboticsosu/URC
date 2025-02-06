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

# Dictionary to store received packets
received_packets = {}
total_packets = None  # Will be set when the first packet arrives

print("Waiting for image packets...")

while True:
    packet = rfm95.receive()
    if packet is not None:
        try:
            seq_end_idx = packet.find(b":")  # Find the position of the colon separator
            sequence_number = int(packet[:seq_end_idx].decode("ascii"))  # Decode the sequence number part


            image_data = packet[seq_end_idx + 1:]


            received_packets[sequence_number] = image_data

            print(f"Received packet {sequence_number}")

            # Set total_packets when the first packet arrives
            if total_packets is None:
                total_packets = sequence_number + 1  # This assumes the first packet gives us the total number of packets


            if len(received_packets) == total_packets:
                print("All packets received, reassembling image...")

                # Sort the packets by sequence number
                sorted_packets = [received_packets[i] for i in range(total_packets)]

                # Join the image chunks into a single byte array
                full_image_data = b''.join(sorted_packets)

                # Write the full image data to a .jpg file
                with open("output_image.jpg", mode = 'wt') as img_file:
                    img_file.write(full_image_data)

                print("Image saved as 'received_image.jpg'")

                break  # Exit the loop after saving the image

        except Exception as e:
            print(f"Error processing packet: {e}")  # Print the actual error message


    time.sleep(0.1)  # Delay to avoid spamming the CPU
