#include <atomic>
#include <iostream>
#include <string>

#include "driverStation.h"
#include "robotActuation.h"
#include "robotControl.h"
#include "robotSerial.h"
#include "robotState.h"
#include "util.h"

#define DS_TIMEOUT_MS 1000

std::atomic<bool> shutdown_flag(false);

void handle_shutdown_signal(int signal) { shutdown_flag = true; }

bool handleDsMessages(DsCommunicator &dsComms, RobotControl &control) {
    SerialPacket dsPacket = {0};
    bool anyMessageRecv = false;
    while (dsComms.readNextMessage(&dsPacket)) {
        anyMessageRecv = true;
        SerialPacketType type = dsPacket.GetType();
        std::cout << "Recieved a serial packet" << std::endl;

        if (type == PACKET_HEARTBEAT) {
            control.handleDsHeartbeatPacket(dsPacket);
        } else if (type == PACKET_GAMEPAD) {
            GamepadPacket gamepadPacket(dsPacket);
            control.handleGamepadPacket(gamepadPacket);
            std::cout << "Received gamepad packet" << std::endl;
        }
         else {
             // Handle packets from the base station that aren't expected here
             std::cout << "Received unexpected radio message type: " << +type << std::endl;
        }
    }

    return anyMessageRecv;
}

int main(int argc, char *argv[]) {
    std::cout << "Initializing robot code " << std::endl;
    DsCommunicator dsComms("/dev/ttyACM1", 115200);
    RobotActuation rp2040("/dev/ttyACM2", 115200);
    RobotControl control;
    control.getRobotState().setMode(ROBOT_MODE_TELEOP); 
    std::cout << "Robot code initialized!" << std::endl;

#ifndef _WIN32
    struct sigaction act;
    act.sa_handler = handle_shutdown_signal;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTERM, &act, NULL);
#endif

    // REMOVED: lastSentDsHearbeat variable (no longer needed)
    uint64_t lastDsMessageRx = getUnixTimeMs();
    while (!shutdown_flag) {
        uint64_t currentTime = getUnixTimeMs();

        // Driver Station Comms
        if (dsComms.isConnected()) {
            // REMOVED: Heartbeat sending to driverstation
            // We only receive from driverstation, never send back

            if (handleDsMessages(dsComms, control)) {
                lastDsMessageRx = getUnixTimeMs();
            } else if (currentTime - lastDsMessageRx > DS_TIMEOUT_MS) {
                std::cout << "DS has not sent a message for "
                          << (currentTime - lastDsMessageRx)
                          << "ms. Killing connection" << std::endl;
                lastDsMessageRx = currentTime;
                control.disableRobot();
            }
        } else {
            control.disableRobot();
            lastDsMessageRx = currentTime;
        }

        // RP2040 Comms
        control.sendStateToRP2040(&rp2040);
        rp2040.run(true);
        dsComms.run(false);

        if (rp2040.isConnected()) {
            SerialPacket packet = {0};
            while (rp2040.readNextMessage(&packet)) {
                if (packet.GetType() == PACKET_LOG) {
                    std::string msg = packet.GetLogMessage();

                    std::cout << "RP2040 LOG: " << msg << std::endl;
                } else {
                    std::cout << "Received unknown message type from RP2040: "
                              << +packet.GetType() << std::endl;
                }
            }
        }
    }

    std::cout << "Robot code shutdown. Goodbye!" << std::endl;

    return 0;
}