#pragma once

#include <stdint.h>

enum RobotMode {
  ROBOT_MODE_DISABLED = 0,
  ROBOT_MODE_TELEOP = 1,
  ROBOT_MODE_AUTO_EXCAVATE = 2,
};

class RobotState {
private:
  // These values are 0 - 4095 (12 bits). Thse actual angle is loc*360/4096
  int8_t deployMotor;

  // All motors are -100 to 100
  // Driver motors
  int8_t flMotor = 0;
  int8_t frMotor = 0;
  int8_t brMotor = 0;
  int8_t blMotor = 0;

  int8_t intakeMotor = 0;
  int8_t dumpMotor = 0;

  RobotMode robotMode = ROBOT_MODE_DISABLED;

public:

  void setMode(RobotMode enabled);
  void setDrive(int8_t left, int8_t right);
  void setDump(int8_t speed);
  void setIntake(int8_t speed);
  void setDeploy(int8_t value);

  int8_t getDriveFrontLeft();
  int8_t getDriveFrontRight();
  int8_t getDriveBackLeft();
  int8_t getDriveBackRight();
  int8_t getDump();
  int8_t getIntake();
  int8_t getDeploy();
  bool isRobotEnabled();
  RobotMode getRobotMode();
};
