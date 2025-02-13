# SPDX-FileCopyrightText: 2023 Kattni Rembor for Adafruit Industries
# SPDX-License-Identifier: MIT
import board
import digitalio
import keypad
import adafruit_rfm9x
import time
# Set up button using keypad module.
button = keypad.Keys((board.BUTTON,), value_when_pressed=False)
# Define radio frequency in MHz. Must match your
# module. Can be a value like 915.0, 433.0, etc.
RADIO_FREQ_MHZ = 915.0
# Define Chip Select and Reset pins for the radio module.
CS = digitalio.DigitalInOut(board.RFM_CS)
RESET = digitalio.DigitalInOut(board.RFM_RST)
# Initialise RFM95 radio
rfm95 = adafruit_rfm9x.RFM9x(board.SPI(), CS, RESET, RADIO_FREQ_MHZ)
# This is the packet to move the motor
packet = [0xbe, 0xef, 0x02, 0x01, 0x64, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
while True:
    print(bytes(packet))
    time.sleep(0.5)
    # This just sends  the info in a loop to avoid if statement issues
    
    rfm95.send(bytes(packet))
    button_press = button.events.get()
    if button_press:
        if button_press.pressed:
            rfm95.send(bytes("button", "UTF-8"))