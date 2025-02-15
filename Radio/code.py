import board
import digitalio
import adafruit_rfm9x
import usb_cdc

RADIO_FREQ_MHZ = 915.0
RADIO_RECV_TIMEOUT = 0.2
ACK_PACKETS=False

# Initialise RFM95 radio
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)

serial = usb_cdc.console

while True:
    # send full message, then wait to see if there is anything to be received
    if serial.in_waiting > 0:
        serial_rx = serial.read(serial.in_waiting)
        if ACK_PACKETS:
            rfm95.send_with_ack(serial_rx)
        else:
            rfm95.send(serial_rx,keep_listening=True)

    packet = rfm95.receive(timeout=RADIO_RECV_TIMEOUT,with_ack=ACK_PACKETS)
    if packet is not None:
        serial.write(packet)