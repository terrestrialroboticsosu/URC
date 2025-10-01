#include "robotState.h"

void RobotState::setDrive(int8_t left, int8_t right) {
  leftMotor = left;
  rightMotor = right;
}

void RobotState::setMode(RobotMode mode) { robotMode = mode; }
int8_t RobotState::getDriveLeft() { return robotMode ? leftMotor : 0; }
int8_t RobotState::getDriveRight() { return robotMode ? rightMotor : 0; }
bool RobotState::isRobotEnabled() { return robotMode != ROBOT_MODE_DISABLED; }
RobotMode RobotState::getRobotMode() {return robotMode;}
