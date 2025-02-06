#include "robotState.h"

void RobotState::setDrive(int8_t left, int8_t right) {
  flMotor = left;
  blMotor = left;
  frMotor = right;
  brMotor = right;
}

void RobotState::setMode(RobotMode mode) { robotMode = mode; }
void RobotState::setIntake(int8_t speed) { intakeMotor = speed; }
void RobotState::setDump(int8_t speed) { dumpMotor = speed; }
void RobotState::setDeploy(int8_t value) { deployMotor = value; }
int8_t RobotState::getDriveFrontLeft() { return robotMode ? flMotor : 0; }
int8_t RobotState::getDriveFrontRight() { return robotMode ? frMotor : 0; }
int8_t RobotState::getDriveBackLeft() { return robotMode ? blMotor : 0; }
int8_t RobotState::getDriveBackRight() { return robotMode ? brMotor : 0; }
int8_t RobotState::getDump() { return robotMode ? dumpMotor : 0; }
int8_t RobotState::getIntake() { return robotMode ? intakeMotor : 0; }
int8_t RobotState::getDeploy() { return robotMode ? deployMotor : 0; } 
bool RobotState::isRobotEnabled() { return robotMode; }
RobotMode RobotState::getRobotMode() {return robotMode;}
