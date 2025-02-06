#pragma once

#include "asio.hpp"
#include "robotSerial.h"
#include <queue>
#include <cstdint>


class RobotActuation : public RobotSerial
{

public:
    RobotActuation(std::string port, unsigned int baud_rate);
    void sendHeartbeat();
    void sendDriveMotors(int8_t frontLeftMotor, int8_t frontRightMotor, int8_t backLeftMotor, int8_t backRightMotor);
    void sendIntakeMotor(int8_t intakeMotorSpeed);
    void sendDumpMotor(int8_t dumpMotorSpeed);
    void sendDeployControl(int8_t angle);
};
