#pragma once
#include "driverStation.h"
#include "robotActuation.h"
#include "robotSerial.h"
#include "robotState.h"

class RobotControl{
private:
    RobotState currentState;
    RobotState lastStateSent;

    // Variables to keep track of how long since we have sent something to the RP2040
    uint64_t lastHeartbeat = 0;
    uint64_t lastDriveCmd = 0;
    uint64_t lastDumpCmd = 0;
    uint64_t lastIntakeCmd = 0;
    uint64_t lastDeployCmd = 0;

public:

    void sendStateToRP2040(RobotActuation *rp2040);

    void handleGamepadPacket(GamepadPacket packet);

    void handleDsHeartbeatPacket(SerialPacket packet);

    void disableRobot();

    RobotState& getRobotState();

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
