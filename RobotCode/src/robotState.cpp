#include "robotState.h"

void RobotState::setDrive(int8_t left, int8_t right) {
  flMotor = left;
  blMotor = left;
  frMotor = right;
  brMotor = right;
}

void RobotState::setEnabled(bool enabled) { robotEnabled = enabled; }
void RobotState::setIntake(int8_t speed) { intakeMotor = speed; }
void RobotState::setDump(int8_t speed) { dumpMotor = speed; }
void RobotState::setDeploy(int8_t value) { deployMotor = value; }
int8_t RobotState::getDriveFrontLeft() { return robotEnabled ? flMotor : 0; }
int8_t RobotState::getDriveFrontRight() { return robotEnabled ? frMotor : 0; }
int8_t RobotState::getDriveBackLeft() { return robotEnabled ? blMotor : 0; }
int8_t RobotState::getDriveBackRight() { return robotEnabled ? brMotor : 0; }
int8_t RobotState::getDump() { return robotEnabled ? dumpMotor : 0; }
int8_t RobotState::getIntake() { return robotEnabled ? intakeMotor : 0; }
int8_t RobotState::getDeploy() { return robotEnabled ? deployMotor : 0; } 
bool RobotState::isRobotEnabled() { return robotEnabled; }
RobotMode RobotState::getRobotMode() { return robotMode; }