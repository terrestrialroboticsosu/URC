#include "robotActuation.h"
#include "robotSerial.h"
#include <iostream>

RobotActuation::RobotActuation(std::string port, unsigned int baud_rate) : RobotSerial(port, baud_rate) {
}

void RobotActuation::sendHeartbeat() {
    SerialPacket packet = {0xBE, 0xEF};
    packet.portions.messageType = PACKET_HEARTBEAT;
    enqueueMessage(&packet);
}

void RobotActuation::sendDriveMotors(int8_t leftMotor, int8_t rightMotor) {
    SerialPacket packet = {0xBE, 0xEF};

    packet.portions.messageType = 0x81;

    //need to change this so that 
    packet.portions.data[0] = leftMotor;
    packet.portions.data[1] = rightMotor;

    enqueueMessage(&packet);
}

