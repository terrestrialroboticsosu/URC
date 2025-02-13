#pragma once
#include <asio.hpp>
#include <string>
#include "robotSerial.h"

struct GamepadPacket { 
    SerialPacket packet;

    GamepadPacket(SerialPacket _packet);

    int8_t getLeftStickY();
    int8_t getRightStickY();
    int8_t getLeftTrigger();
    int8_t getRightTrigger();

    bool isButtonAPressed();
    bool isButtonBPressed();
    bool isDpadUp();
    bool isDpadDown();
    bool isLeftBumperPressed();
    bool isRightBumperPressed();
};

class DsCommunicator : public RobotSerial
{
public:
    DsCommunicator(std::string port, unsigned int baud_rate);
};
