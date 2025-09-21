#include "driverStation.h"
#include <asio.hpp>

DsCommunicator::DsCommunicator(std::string port, unsigned int baud_rate)
    : RobotSerial(port, baud_rate) {}

GamepadPacket::GamepadPacket(SerialPacket _packet) : packet(_packet) {}

// gamepad packets start at 1, robot state is the 0th byte
int8_t GamepadPacket::getLeftStickY() { return packet.portions.data[1]; }

int8_t GamepadPacket::getRightStickY() { return packet.portions.data[2]; }

int8_t GamepadPacket::getLeftTrigger() { return packet.portions.data[3]; }

int8_t GamepadPacket::getRightTrigger() { return packet.portions.data[4]; }

bool GamepadPacket::isButtonAPressed() { return packet.portions.data[5]; }

bool GamepadPacket::isButtonBPressed() { return packet.portions.data[6]; }

bool GamepadPacket::isDpadUp() { return packet.portions.data[7]; }

bool GamepadPacket::isDpadDown() { return packet.portions.data[8]; }

bool GamepadPacket::isLeftBumperPressed() { return packet.portions.data[9]; } //TODO: verify if needed

bool GamepadPacket::isRightBumperPressed() { return packet.portions.data[10]; } //TODO: verify if neededs
