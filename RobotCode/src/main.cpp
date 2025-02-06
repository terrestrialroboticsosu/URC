#include <iostream>
#include <string>
#include <thread>
#include <atomic>

#include "driverStation.h"
#include "robotActuation.h"
#include "robotControl.h"
#include "robotState.h"
#include "util.h"

#define DS_HEARTBEAT_RATE_MS 500
#define DS_TIMEOUT_MS 3000

std::atomic<bool> shutdown_flag(false);

void handle_shutdown_signal(int signal) { shutdown_flag = true; }

bool handleDsMessages(DsCommunicator &dsComms, RobotControl &control) {
    DsPacket dsPacket;
    bool anyMessageRecv = false;
    while (dsComms.receiveMessage(&dsPacket)) {
        anyMessageRecv = true;
        DsPacketType type = dsPacket.getType();

        if (type == DS_PACKET_HEARTBEAT) {
            DsHeartbeatPacket heartbeatPacket(dsPacket);
            control.handleDsHeartbeatPacket(heartbeatPacket);
        } else if (type == DS_PACKET_GAMEPAD) {
            GamepadPacket gamepadPacket(dsPacket);
            control.handleGamepadPacket(gamepadPacket);
        }
    }

    return anyMessageRecv;
}

int main(int argc, char *argv[]) {
    std::cout << "Initializing robot code " << std::endl;
    DsCommunicator dsComms("/dev/ttyTBD", 115200); // TODO: figure out wtf this should be
    RobotActuation rp2040("/dev/ttyACM0", 115200);
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
        dsComms.update();
        if (dsComms.isConnected()) {
            if (currentTime - lastSentDsHearbeat > DS_HEARTBEAT_RATE_MS) {
                dsComms.sendHeartbeat(control.getRobotState().isRobotEnabled(), rp2040.isConnected(),
                                      control.getRobotState().getRobotMode());
                lastSentDsHearbeat = currentTime;
            }

            if (handleDsMessages(dsComms, control)) {
                lastDsMessageRx = getUnixTimeMs();
            } else if (currentTime - lastDsMessageRx > DS_TIMEOUT_MS) {
                std::cout << "DS has not sent a message for " << (currentTime - lastDsMessageRx)
                          << "ms. Killing connection" << std::endl;
                dsComms.close();
            }
        } else {
            control.disableRobot();
            lastDsMessageRx = currentTime;
        }

        // RP2040 Comms
        control.sendStateToRP2040(&rp2040);
        rp2040.run();

        if (rp2040.isConnected()) {
            SerialPacket packet = {0};
            while (rp2040.readNextMessage(&packet)) {
                if (packet.GetType() == SERIAL_PACKET_LOG) {
                    std::string msg = packet.GetLogMessage();

                    std::cout << "RP2040 LOG: " << msg << std::endl;
                } else if (packet.GetType() == SERIAL_MSG_TYPE_INTAKE_POS_TLM) {
                    int pos = packet.GetIntakePos();

                    std::cout << "INTAKE POS: " << pos << std::endl;
                    dsComms.sendIntakePos(pos);
                } else {
                    std::cout << "Received unknown message type from RP2040: " << +packet.GetType() << std::endl;
                }
            }
        }
    }

    std::cout << "Shutting down robot code..." << std::endl;
    dsComms.close();
    std::cout << "Robot code shutdown. Goodbye!" << std::endl;

    return 0;
}
