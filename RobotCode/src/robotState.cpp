#include "robotState.h"

void RobotState::setDrive(int8_t left, int8_t right) {
  flMotor = left;
  blMotor = left;
  frMotor = right;
  brMotor = right;
}

void RobotState::setMode(RobotMode mode) { robotMode = mode; }
int8_t RobotState::getDriveLeft() { return robotMode ? flMotor : 0; }
int8_t RobotState::getDriveRight() { return robotMode ? frMotor : 0; }
bool RobotState::isRobotEnabled() { return robotMode; }
RobotMode RobotState::getRobotMode() {return robotMode;}
