# URC Software
Software for the [Ohio State Terrestrial Robotics](https://org.osu.edu/firstrobotics/urc/).

## Architecture

The software is split into four primary components:

- **Driver Station (Python)**: Allows operators to remotely control the robot. This software is run on a laptop that connects to the robot's Raspberry Pi via Wifi.
- **Radio (Arduino)**: LoRa-based communication using arduino. The system uses an RH_RF95 LoRa module to send and recieve data packets  
- **HAL(C)**: the Hardware Abstraction Layer which interacts with hardware such as motors and sensors. Runs on the RP2040 feather board (HAL board).
- **RobotCode(C++)**: High level robot code which handles networking, autonomy, computer vision. Communicates with the Driverstation via a TCP connection and communicates with the RP2040 HAL board via a USB serial port.


## File Structure

Documentation: contains relevant documentation files
DriverStation: contains the code to run the driver station
Radio: contains the code to run the RP2040 RFM Feather radio board. 
HAL: contains the code to run on the RP2040 Feather board.
RobotCode: contains the code which runs on the raspberry pi.
TestScripts: Utility scripts which can be used for testing the robot's software.
