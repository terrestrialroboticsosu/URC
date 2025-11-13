#include "robotSerial.h"
#include <iostream>

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

bool RobotSerial::readNextMessage(SerialPacket *packet) {
    if (!serial.is_open()) return false;

    while (recvData.size() >= 4) { // minimum header length
        uint8_t syncByte1 = recvData.front(); recvData.pop();
        uint8_t syncByte2 = recvData.front(); recvData.pop();
        if (syncByte1 != 0xBE || syncByte2 != 0xEF) {
            std::cout << portName << " sent incorrect sync bytes: "
                      << (int)syncByte1 << " and " << (int)syncByte2 << std::endl;
            continue;
        }

        if (recvData.size() < 2) return false;
        uint8_t msg_len_L = recvData.front(); recvData.pop();
        uint8_t msg_len_H = recvData.front(); recvData.pop();
        uint16_t msg_len = msg_len_L | (msg_len_H << 8);

        if (recvData.size() < msg_len) return false;

        packet->portions.messageType = recvData.front(); recvData.pop();
        for (int i = 0; i < msg_len - 1; i++) {
            packet->portions.data[i] = recvData.front();
            recvData.pop();
        }
        return true;
    }

    return false;
}

void RobotSerial::sendHeartbeat(int robotState, bool rp2040Connected) {
    uint8_t data[2] = { static_cast<uint8_t>(robotState), static_cast<uint8_t>(rp2040Connected) };
    uint8_t messageType = PACKET_HEARTBEAT;

    std::vector<uint8_t> payload = { messageType, data[0], data[1] };
    std::vector<uint8_t> fullPacket = {
        0xBE, 0xEF,
        static_cast<uint8_t>(payload.size() & 0xFF),
        static_cast<uint8_t>((payload.size() >> 8) & 0xFF)
    };
    fullPacket.insert(fullPacket.end(), payload.begin(), payload.end());

    serial.write_some(asio::buffer(fullPacket.data(), fullPacket.size()));
}

int RobotSerial::sendCurrentQueue(bool /*unused*/) {
    if (byteQueueFull && !serialTransmit) {
        serialTransmit = true;
        serial.async_write_some(asio::buffer(outgoingBytes, SERIAL_MES_LEN),
                                [this](const asio::error_code &error,
                                       std::size_t bytes_transferred) {
                                    sendBytesHandler(error, bytes_transferred);
                                });
    }
    if (!byteQueueFull && !serialTransmit && !outgoingQueue.empty()) {
        SerialPacket *packet = &outgoingQueue.front();
        std::copy(std::begin(packet->packet), std::end(packet->packet), outgoingBytes);
        outgoingQueue.pop();
        byteQueueFull = true;
    }
    return 0;
}

void RobotSerial::sendBytesHandler(const asio::error_code &error, std::size_t bytes_transferred) {
    positonOfNextOutgoingByte += bytes_transferred;
    if (positonOfNextOutgoingByte >= SERIAL_MES_LEN) {
        byteQueueFull = false;
        serialTransmit = false;
        positonOfNextOutgoingByte = 0;
    }
}

void RobotSerial::onRead(const asio::error_code &ec, size_t len) {
    if (!ec) {
        for (size_t i = 0; i < len; i++) recvData.push(rxBuf[i]);
        memset(rxBuf, 0, SERIAL_RX_BUF_SIZE);
    } else {
        std::cout << "Failed to read from serial port: " << ec.message() << std::endl;
    }
    reading = false;
}

void RobotSerial::startReading() {
    serial.async_read_some(asio::buffer(rxBuf, SERIAL_RX_BUF_SIZE),
                           [this](const asio::error_code &ec, size_t len) {
                               onRead(ec, len);
                           });
    reading = true;
}

void RobotSerial::run(bool /*unused*/) {
    if (io.stopped()) io.reset();
    sendCurrentQueue(false);
    io.poll();
    if (serial.is_open() && !reading) startReading();
}

bool RobotSerial::isConnected() { return serial.is_open(); }

void RobotSerial::enqueueMessage(SerialPacket *mess) {
    outgoingQueue.push(*mess);
}


