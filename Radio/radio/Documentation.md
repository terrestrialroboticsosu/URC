# Radio Guide

## Setup
At time of usage, the installed libraries inside of the Arduino IDE are as follows:
- SD 1.3.0
- Adafruit GPS Library 1.7.5
- IRremote 4.5.0
- LiquidCrystal I2C 1.1.2
- RadioHead 1.143.1
- Serial Transfer 3.1.5

Select the option 'Adafruit Feather RP2040 RFM' for your board with the correct COM port (seen as a USB on first connect). For debug, the radio.ino is a working version that has LoRa initialized correctly. The updated code with GPS included is the radiogps.ino file. 

## GPS Setup
It has been observed that GPS does not work inside of buildings. We have to test outside to get positioning. Connect the GPS to the RP2040 using wires with the following pin setup. We are using a breadboard to hold the pins in place at the moment, be mindful that you are aligning the pins vertically and not horizontally.
GPS pin setup:

## References
Radiohead integrated usage: https://learn.adafruit.com/feather-rp2040-rfm95/using-the-rfm-9x-radio 
