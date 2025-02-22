#include <atomic>
#include <iostream>
#include <string>

#include "driverStation.h"
#include "robotActuation.h"
#include "robotControl.h"
#include "robotSerial.h"
#include "robotState.h"
#include "util.h"

#define DS_HEARTBEAT_RATE_MS 500
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
    }

    return anyMessageRecv;
}

int main(int argc, char *argv[]) {
    std::cout << "Initializing robot code " << std::endl;
    DsCommunicator dsComms("/dev/ttyACM0", 115200);
    RobotActuation rp2040("/dev/ttyACM1", 115200);
    RobotControl control;
    std::cout << "Robot code initialized!" << std::endl;

#ifndef _WIN32
    struct sigaction act;
    act.sa_handler = handle_shutdown_signal;
    sigaction(SIGINT, &act, NULL);
    sigaction(SIGTERM, &act, NULL);
#endif

    uint64_t lastSentDsHearbeat = 0;
    uint64_t lastDsMessageRx = getUnixTimeMs();
    while (!shutdown_flag) {
        uint64_t currentTime = getUnixTimeMs();

        // Driver Station Comms
        if (dsComms.isConnected()) {
            if (currentTime - lastSentDsHearbeat > DS_HEARTBEAT_RATE_MS) {
                dsComms.sendHeartbeat(control.getRobotState().getRobotMode(),
                                      rp2040.isConnected());
                lastSentDsHearbeat = currentTime;
            }

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
        rp2040.run();
        dsComms.run();

        if (rp2040.isConnected()) {
            SerialPacket packet = {0};
            while (rp2040.readNextMessage(&packet)) {
                if (packet.GetType() == PACKET_LOG) {
                    std::string msg = packet.GetLogMessage();

                    std::cout << "RP2040 LOG: " << msg << std::endl;
                    //} else if (packet.GetType() == PACKET_INTAKE_POS) {
                    // int pos = packet.GetIntakePos();

                    // std::cout << "INTAKE POS: " << pos << std::endl;
                    // dsComms.sendIntakePos(pos);
                    //  intake does not exist anymore rn this code will be
                    //  removed soon
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
