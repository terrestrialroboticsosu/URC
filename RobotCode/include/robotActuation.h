/**
 * @file robotActuation.h
 * @brief Defines the interface for communicating with the robot's actuation hardware.
 *
 * This file contains the RobotActuation class, which handles sending commands
 * to the RP2040 microcontroller that directly controls motors and other actuators.
 */

#pragma once
#include "asio.hpp"
#include "robotSerial.h"
#include <queue>
#include <cstdint>

/**
 * @class RobotActuation
 * @brief Manages serial communication for sending actuation commands.
 *
 * This class inherits from RobotSerial and provides specific methods for commanding
 * the robot's motors, such as drive motors.
 */
class RobotActuation : public RobotSerial
{

public:
    /**
     * @brief Construct a new RobotActuation object.
     * @param port The serial port name for the actuation board (e.g., "/dev/ttyACM1").
     * @param baud_rate The communication speed.
     */
    RobotActuation(std::string port, unsigned int baud_rate);

    /**
     * @brief Sends a heartbeat packet to the actuation board to maintain connection.
     */
    void sendHeartbeat();

    /**
     * @brief Sends drive motor speed commands.
     * @param leftMotor The speed for the left side motors (-100 to 100).
     * @param rightMotor The speed for the right side motors (-100 to 100).
     */
    void sendDriveMotors(int8_t leftMotor, int8_t rightMotor);
};