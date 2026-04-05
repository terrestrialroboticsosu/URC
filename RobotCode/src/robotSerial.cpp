#include "robotSerial.h"
#include <iostream>
#include <vector>
#include <cstring>

// Constructor
RobotSerial::RobotSerial(std::string port, unsigned int baud_rate)
    : io(), serial(io), portName(port) {
    try {
        serial.open(port);
        serial.set_option(asio::serial_port_base::baud_rate(baud_rate));
        std::cout << "Opened port for " << port << std::endl;
    } catch (asio::system_error &e) {
        std::cout << "Failed to open serial port: " << e.what() << std::endl;
    }
}

    // Read next message
    bool RobotSerial::readNextMessage(SerialPacket *packet) {
        if (!serial.is_open()) return false;

        static bool sawFirstPacket = false;

        while (recvData.size() >= SERIAL_MES_LEN) {

            // Read potential sync bytes
            uint8_t b1 = recvData.front(); recvData.pop();
            uint8_t b2 = recvData.front(); recvData.pop();

            if (b1 == 0xBE && b2 == 0xEF) {

                if (!sawFirstPacket) {
                    // FIRST PACKET — IGNORE IT COMPLETELY
                    sawFirstPacket = true;

                    // Throw away the entire packet (type + data + checksums)
                    recvData.pop(); // messageType
                    for (int i = 0; i < SERIAL_MES_DATA_LEN; i++) recvData.pop();
                    recvData.pop(); // checksum high
                    recvData.pop(); // checksum low

                    continue;
                }

                // SECOND PACKET — THIS IS THE GOOD ONE
                sawFirstPacket = false; // reset for next round

                // Parse packet normally
                packet->portions.messageType = recvData.front(); recvData.pop();
                for (int i = 0; i < SERIAL_MES_DATA_LEN; i++) {
                    packet->portions.data[i] = recvData.front();
                    recvData.pop();
                }

                uint8_t chkH = recvData.front(); recvData.pop();
                uint8_t chkL = recvData.front(); recvData.pop();

                // ---- PRINT THE GOOD PACKET IN ONE HEX STRING ----
                std::cout << "\n=== GOOD DS PACKET RECEIVED ===" << std::endl;

                // Print sync bytes first
                std::cout << "beef";

                // Print messageType
                if (packet->portions.messageType < 0x10) std::cout << "0";
                std::cout << std::hex << (int)packet->portions.messageType;

                // Print data bytes
                for (int i = 0; i < SERIAL_MES_DATA_LEN; i++) {
                    if (packet->portions.data[i] < 0x10) std::cout << "0";
                    std::cout << std::hex << (int)packet->portions.data[i];
                }

                // Print checksums
                if (chkH < 0x10) std::cout << "0";
                std::cout << std::hex << (int)chkH;
                if (chkL < 0x10) std::cout << "0";
                std::cout << std::hex << (int)chkL;

                std::cout << std::dec << std::endl; // reset to decimal
                std::cout << "================================\n" << std::endl;

                // ---- SEND GOOD PACKET TO HAL IMMEDIATELY ----
                enqueueMessage(packet);   // push it to outgoing queue (HAL)

                return true;
            }

            // Not sync bytes → ignore noise
        }

        return false;
    }

// DISABLED: This method is no longer used for driverstation communication
// Only used internally for HAL communication
void RobotSerial::sendHeartbeat(int robotState, bool rp2040Connected) {
    // This method should only be called for HAL communication, not driverstation
    // For one-way LoRa, we don't send anything back to driverstation
    std::cout << "WARNING: sendHeartbeat called but should not send to DS over LoRa" << std::endl;
    
    // If this is called for HAL communication, keep the implementation:
    SerialPacket packet = {0};
    packet.portions.messageType = PACKET_HEARTBEAT;
    packet.portions.data[0] = static_cast<uint8_t>(robotState);
    packet.portions.data[1] = static_cast<uint8_t>(rp2040Connected);

    enqueueMessage(&packet);
}

// Send messages in queue
int RobotSerial::sendCurrentQueue(bool checksum) {
    if (byteQueueFull && !serialTransmit) {
        serialTransmit = true;
        serial.async_write_some(asio::buffer(outgoingBytes, SERIAL_MES_LEN),
                                [this](const asio::error_code &error, std::size_t bytes_transferred) {
                                    sendBytesHandler(error, bytes_transferred);
                                });
    }

    if (!byteQueueFull && !serialTransmit && !outgoingQueue.empty()) {
        SerialPacket *packet = &outgoingQueue.front();
        if (checksum) addChecksum(packet);
        std::copy(std::begin(packet->packet), std::end(packet->packet), outgoingBytes);
        outgoingQueue.pop();
        byteQueueFull = true;
    }

    return 0;
}

// Async write handler
void RobotSerial::sendBytesHandler(const asio::error_code &error, std::size_t bytes_transferred) {
    positonOfNextOutgoingByte += bytes_transferred;
    if (positonOfNextOutgoingByte >= SERIAL_MES_LEN) {
        byteQueueFull = false;
        serialTransmit = false;
        positonOfNextOutgoingByte = 0;
    }
}

// Async read handler
void RobotSerial::onRead(const asio::error_code &ec, size_t len) {
    if (!ec) {
        for (size_t i = 0; i < len; i++) {

            // NEW: print every raw byte received
            std::cout << "[" << portName << "] RX: 0x"
                      << std::hex << (int)rxBuf[i] << std::dec
                      << " (" << (int)rxBuf[i] << ")" << std::endl;
                      
            recvData.push(rxBuf[i]);
        }

        memset(rxBuf, 0, SERIAL_RX_BUF_SIZE);
    } else {
        std::cout << "Failed to read from serial port: " << ec.message() << std::endl;
    }

    reading = false;
}

// Start reading asynchronously
void RobotSerial::startReading() {
    serial.async_read_some(asio::buffer(rxBuf, SERIAL_RX_BUF_SIZE),
                           [this](const asio::error_code &ec, size_t len) { onRead(ec, len); });
    reading = true;
}

// Run Asio loop
void RobotSerial::run(bool checksum) {
    if (io.stopped()) io.reset();
    sendCurrentQueue(checksum);
    io.poll();
    if (serial.is_open() && !reading) startReading();
}

// Connection check
bool RobotSerial::isConnected() { return serial.is_open(); }

// Add message to queue
void RobotSerial::enqueueMessage(SerialPacket *mess) {
    outgoingQueue.push(*mess);
}

// Add checksum
void RobotSerial::addChecksum(SerialPacket *packet) {
    uint16_t checksum = fletcher16(packet->packet, SERIAL_MES_LEN - 2);
    packet->portions.checksumHigh = (checksum >> 8) & 0xFF;
    packet->portions.checksumLow = checksum & 0xFF;
}

// Fletcher-16 checksum
uint16_t RobotSerial::fletcher16(const uint8_t *data, size_t len) {
    uint32_t c0 = 0, c1 = 0;
    while (len) {
        size_t blocklen = len > 5802 ? 5802 : len;
        len -= blocklen;
        do {
            c0 += *data++;
            c1 += c0;
        } while (--blocklen);
        c0 %= 255;
        c1 %= 255;
    }
    return static_cast<uint16_t>((c1 << 8) | c0);
}

// SerialPacket functions
std::string SerialPacket::GetLogMessage() {
    std::string msg;
    for (size_t i = 0; i < SERIAL_MES_DATA_LEN; i++) {
        if (portions.data[i] == 0) break;
        msg.push_back(static_cast<char>(portions.data[i]));
    }
    return msg;
}

SerialPacketType SerialPacket::GetType() {
    return static_cast<SerialPacketType>(portions.messageType);
}