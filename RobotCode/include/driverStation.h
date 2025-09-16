/**
 * @file driverStation.h
 * @brief Defines the structures and classes for interpreting data from the driver station.
 *
 * This file contains the GamepadPacket struct, which provides a high-level interface
 * for accessing gamepad data from a raw SerialPacket.
 */

#pragma once
#include <asio.hpp>
#include <string>
#include "robotSerial.h"

/**
 * @struct GamepadPacket
 * @brief A wrapper for a SerialPacket that provides easy access to gamepad controls.
 *
 * This struct takes a raw SerialPacket and exposes methods to get the state of
 * various joystick axes and buttons, such as stick positions and button presses.
 */
struct GamepadPacket {
    SerialPacket packet; /**< The raw serial packet containing the gamepad data. */

    /**
     * @brief Construct a new GamepadPacket object.
     * @param _packet The raw SerialPacket received from the driver station.
     */
    GamepadPacket(SerialPacket _packet);

    /** @return The Y-axis value of the left joystick (-100 to 100). */
    int8_t getLeftStickY();

    /** @return The Y-axis value of the right joystick (-100 to 100). */
    int8_t getRightStickY();

    /** @return The value of the left trigger (0 to 100). */
    int8_t getLeftTrigger();

    /** @return The value of the right trigger (0 to 100). */
    int8_t getRightTrigger();

    /** @return True if the 'A' button is pressed, false otherwise. */
    bool isButtonAPressed();

    /** @return True if the 'B' button is pressed, false otherwise. */
    bool isButtonBPressed();

    /** @return True if the D-pad Up is pressed, false otherwise. */
    bool isDpadUp();

    /** @return True if the D-pad Down is pressed, false otherwise. */
    bool isDpadDown();

    /** @return True if the left bumper is pressed, false otherwise. */
    bool isLeftBumperPressed();

    /** @return True if the right bumper is pressed, false otherwise. */
    bool isRightBumperPressed();
};

/**
 * @class DsCommunicator
 * @brief Inherits from RobotSerial to handle communication with the Driver Station.
 *
 * This class is a specialized version of RobotSerial, intended to manage the
 * serial connection to the Driver Station.
 */
class DsCommunicator : public RobotSerial
{
public:
    /**
     * @brief Construct a new DsCommunicator object.
     * @param port The serial port name (e.g., "/dev/ttyACM0").
     * @param baud_rate The communication speed in bits per second.
     */
    DsCommunicator(std::string port, unsigned int baud_rate);
};