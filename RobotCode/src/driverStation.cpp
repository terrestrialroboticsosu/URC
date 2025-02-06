#include "driverStation.h"
#include <asio.hpp>

DsCommunicator::DsCommunicator(std::string port, unsigned int baud_rate) : RobotSerial(port, baud_rate) {
}

DsPacketType DsPacket::getType() { return (DsPacketType)data[0]; }

GamepadPacket::GamepadPacket(DsPacket _packet) : packet(_packet) {}

int8_t GamepadPacket::getLeftStickY() { return packet.data[1]; }

int8_t GamepadPacket::getRightStickY() { return packet.data[3]; }

int8_t GamepadPacket::getLeftTrigger() { return packet.data[7]; }

int8_t GamepadPacket::getRightTrigger() { return packet.data[8]; }

bool GamepadPacket::isButtonAPressed() { return packet.data[5] & 0x40; }

bool GamepadPacket::isButtonBPressed() { return packet.data[5] & 0x80; }

bool GamepadPacket::isDpadUp() { return packet.data[5] & 0x04; }

bool GamepadPacket::isDpadDown() { return packet.data[5] & 0x08; }

bool GamepadPacket::isLeftBumperPressed() { return packet.data[6] & 0x10; }

bool GamepadPacket::issRightBumperPressed() { return packet.data[6] & 0x20; }

DsHeartbeatPacket::DsHeartbeatPacket(DsPacket _packet) : packet(_packet) {}

bool DsHeartbeatPacket::IsRobotEnabled() { return packet.data[1] != 0; }
