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
    void RobotActuation::sendDriveMotors(int8_t leftMotor, int8_t rightMotor);
};
