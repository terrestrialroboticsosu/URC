/**
 * @file robotState.h
 * @brief Contains the definition of the robot's state.
 *
 * This file defines the RobotState class, which encapsulates all the critical
 * state information of the robot, such as its operating mode and motor speeds.
 */

#pragma once

#include <stdint.h>

/**
 * @enum RobotMode
 * @brief Defines the possible operating modes of the robot.
 */
enum RobotMode {
  ROBOT_MODE_DISABLED = 0,      /**< The robot is disabled and will not move. */
  ROBOT_MODE_TELEOP = 1,        /**< The robot is in teleoperated (manual) mode. */
  ROBOT_MODE_AUTO = 2,          /**< The robot is in an autonomous driving mode. */
};

/**
 * @class RobotState
 * @brief A class to hold and manage the state of the robot.
 *
 * This class stores information like motor speeds and the current robot mode.
 * It provides getter and setter methods to safely access and modify the state.
 */
class RobotState {
private:
  // Motor values are from (-100 to 100)
  int8_t leftMotor = 0;  /**< Speed of the left drive motors (-100 to 100). */
  int8_t rightMotor = 0; /**< Speed of the right drive motors (-100 to 100). */

  RobotMode robotMode = ROBOT_MODE_DISABLED; /**< The current operating mode of the robot. */

public:

  /**
   * @brief Sets the operating mode of the robot.
   * @param mode The desired RobotMode.
   */
  void setMode(RobotMode mode);

  /**
   * @brief Sets the speed of the drive motors.
   * @param left The speed for the left motors.
   * @param right The speed for the right motors.
   */
  void setDrive(int8_t left, int8_t right);

  /**
   * @brief Gets the current speed of the left drive motors.
   * @return The speed value, or 0 if the robot is disabled.
   */
  int8_t getDriveLeft();

  /**
   * @brief Gets the current speed of the right drive motors.
   * @return The speed value, or 0 if the robot is disabled.
   */
  int8_t getDriveRight();

  /**
   * @brief Checks if the robot is currently enabled.
   * @return True if the robot is not in ROBOT_MODE_DISABLED.
   */
  bool isRobotEnabled();

  /**
   * @brief Gets the current robot mode.
   * @return The current RobotMode enum value.
   */
  RobotMode getRobotMode();
};