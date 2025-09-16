/**
 * @file robotControl.h
 * @brief Contains the central logic for controlling the robot's behavior.
 *
 * This file defines the RobotControl class, which acts as the "brain" of the robot.
 * It takes inputs from the driver station, processes them, and sends the appropriate
 * commands to the actuation layer.
 */

#pragma once
#include "driverStation.h"
#include "robotActuation.h"
#include "robotSerial.h"
#include "robotState.h"

/**
 * @class RobotControl
 * @brief Manages the robot's state and translates DS inputs to actuation commands.
 *
 * This class holds the current state of the robot, handles incoming packets
 * from the Driver Station (gamepad, heartbeats), and decides what commands
 * to send to the RP2040 board.
 */
class RobotControl{
private:
    RobotState currentState;  /**< The desired state of the robot. */
    RobotState lastStateSent; /**< The last state that was sent to the RP2040. */

    uint64_t lastHeartbeat = 0; /**< Timestamp of the last heartbeat sent to the RP2040. */
    uint64_t lastDriveCmd = 0;  /**< Timestamp of the last drive command sent. */

public:

    /**
     * @brief Sends the current robot state to the RP2040 actuation board.
     *
     * This method checks for changes in state and timers to decide whether
     * to send new motor commands or a heartbeat.
     * @param rp2040 A pointer to the RobotActuation object.
     */
    void sendStateToRP2040(RobotActuation *rp2040);

    /**
     * @brief Processes a GamepadPacket and updates the robot's state.
     * @param packet The gamepad packet received from the driver station.
     */
    void handleGamepadPacket(GamepadPacket packet);

    /**
     * @brief Processes a heartbeat packet from the driver station.
     * @param packet The heartbeat packet.
     */
    void handleDsHeartbeatPacket(SerialPacket packet);

    /**
     * @brief Sets the robot's mode to disabled as a safety measure.
     */
    void disableRobot();

    /**
     * @brief Provides access to the current state of the robot.
     * @return A reference to the RobotState object.
     */
    RobotState& getRobotState();

    /************************************************* 
     * Future Code
     * (to be implemented)
    **************************************************/

    // int robotStartup();

    // int robotRunStateMachine();

    // int runStartupState();

    // int runStopState();

    // int runTeleopState();

    // int runAutoState();

    // int runPanicState();

    // int setDriveValues(int8_t flVal, int8_t frVal, int8_t blVal, int8_t brVal);

    // int setIntakeSpeed(int8_t val);

    // int setDumpSpeed(int8_t val); 

    // int setIntakePosition(int8_t val);
};

