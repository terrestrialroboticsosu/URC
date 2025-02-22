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


  // All motors are -100 to 100
  // Driver motors
  int8_t leftMotor = 0;
  int8_t rightMotor = 0;

  RobotMode robotMode = ROBOT_MODE_TELEOP;

public:

  void setMode(RobotMode enabled);
  void setDrive(int8_t left, int8_t right);

  int8_t getDriveLeft();
  int8_t getDriveRight();

  bool isRobotEnabled();
  RobotMode getRobotMode();
};
