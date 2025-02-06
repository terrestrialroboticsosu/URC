#include "robotActuation.h"
#include "robotSerial.h"
#include <iostream>

RobotActuation::RobotActuation(std::string port, unsigned int baud_rate) : RobotSerial(port, baud_rate) {
}

void RobotActuation::sendDriveMotors(int8_t frontLeftMotor, int8_t frontRightMotor, int8_t backLeftMotor,
                                     int8_t backRightMotor) {
    SerialPacket packet = {0xBE, 0xEF};

    packet.portions.messageType = 0x81;

    packet.portions.data[0] = frontLeftMotor;
    packet.portions.data[1] = frontRightMotor;
    packet.portions.data[2] = backLeftMotor;
    packet.portions.data[3] = backRightMotor;

    enqueueMessage(&packet);
}

void RobotActuation::sendIntakeMotor(int8_t intakeMotorSpeed) {
    SerialPacket packet = {0xBE, 0xEF};

    packet.portions.messageType = 0x83;

    packet.portions.data[0] = intakeMotorSpeed;

    enqueueMessage(&packet);
}

void RobotActuation::sendDumpMotor(int8_t dumpMotorSpeed) {
    SerialPacket packet = {0xBE, 0xEF};

    packet.portions.messageType = 0x84;

    packet.portions.data[0] = dumpMotorSpeed;

    enqueueMessage(&packet);
}

void RobotActuation::sendDeployControl(int8_t angle) {
    SerialPacket packet = {0xBE, 0xEF};

    packet.portions.messageType = 0x82;

    packet.portions.data[0] = angle;

    enqueueMessage(&packet);
}

