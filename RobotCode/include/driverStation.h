#pragma once
#include <queue>
#include <asio.hpp>
#include <string>
#include "robotSerial.h"
#include <iostream>

// Defines length for each packet type
#define MESS_0x01_LEN 11

enum DsPacketType {
    DS_PACKET_HEARTBEAT = 0x01,
    DS_PACKET_GAMEPAD = 0x02,
};

struct DsPacket {
    uint8_t data[MESS_0x01_LEN];

    DsPacketType getType();
};

struct GamepadPacket { 
    DsPacket packet;

    GamepadPacket(DsPacket _packet);

    int8_t getLeftStickY();
    int8_t getRightStickY();
    int8_t getLeftTrigger();
    int8_t getRightTrigger();

    bool isButtonAPressed();
    bool isButtonBPressed();
    bool isDpadUp();
    bool isDpadDown();
    bool isLeftBumperPressed();
    bool issRightBumperPressed();
};

struct DsHeartbeatPacket { 
    DsPacket packet;

    DsHeartbeatPacket(DsPacket _packet);

    bool IsRobotEnabled();
};

class DsCommunicator : public RobotSerial
{
public:
    DsCommunicator(std::string port, unsigned int baud_rate);

private:
};
