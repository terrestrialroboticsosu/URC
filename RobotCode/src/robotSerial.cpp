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
    if (!serial.is_open()) {
        return false;
    }

    while (recvData.size() >= SERIAL_MES_LEN) {
        uint8_t syncByte1 = recvData.front();
        recvData.pop();
        uint8_t syncByte2 = recvData.front();
        recvData.pop();
        if (syncByte1 == 0xBE && syncByte2 == 0xEF) {
            packet->portions.messageType = recvData.front();
            recvData.pop();

            for (int i = 0; i < SERIAL_MES_DATA_LEN; i++) {
                packet->portions.data[i] = recvData.front();
                recvData.pop();
            }

            // remove checksums
            recvData.pop();
            recvData.pop();
            return true;
        } else {
            std::cout << portName << " sent incorrect sync bytes: "
                      << std::to_string((int)syncByte1) << " and "
                      << std::to_string((int)syncByte2) << std::endl;
        }
    }

    return false;
}

void RobotSerial::sendHeartbeat(int robotState, bool rp2040Connected) {
    uint8_t data[2] = { static_cast<uint8_t>(robotState), static_cast<uint8_t>(rp2040Connected) };
    uint8_t packetLen = sizeof(data) + 1; // messageType + data
    uint8_t messageType = PACKET_HEARTBEAT;

    // Build packet like ds.py expects
    std::vector<uint8_t> payload;
    payload.push_back(messageType);
    payload.insert(payload.end(), data, data + sizeof(data));

    // Add header and CRC like ds.py
    uint16_t crc = gen_crc16(payload); // you’ll need to add the same gen_crc16 function as in ds.py
    std::vector<uint8_t> fullPacket = {0xBE, 0xEF, static_cast<uint8_t>(packetLen & 0xFF),
                                       static_cast<uint8_t>((packetLen >> 8) & 0xFF)};
    fullPacket.insert(fullPacket.end(), payload.begin(), payload.end());
    fullPacket.push_back(crc & 0xFF);
    fullPacket.push_back((crc >> 8) & 0xFF);

    serial.write_some(asio::buffer(fullPacket.data(), fullPacket.size()));
}

int RobotSerial::sendCurrentQueue(bool checksum) {
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

        if (checksum)
            addChecksum(packet);

        std::copy(std::begin(packet->packet), std::end(packet->packet),
                  outgoingBytes);

        outgoingQueue.pop();
        byteQueueFull = true;
    }
    return 0;
}

void RobotSerial::sendBytesHandler(const asio::error_code &error,
                                   std::size_t bytes_transferred) {
    // std::cout << "Sent " << unsigned(bytes_transferred) << " bytes" <<
    // std::endl; std::cout << "Error code: " << error.value() << std::endl;
    positonOfNextOutgoingByte += bytes_transferred;
    if (positonOfNextOutgoingByte >= SERIAL_MES_LEN) {
        byteQueueFull = false;
        serialTransmit = false;
        positonOfNextOutgoingByte = 0;
    }
}

void RobotSerial::onRead(const asio::error_code &ec, size_t len) {
    if (ec.value() == 0) {
        for (size_t i = 0; i < len; i++) {
            this->recvData.push(rxBuf[i]);
        }

        memset(rxBuf, 0, SERIAL_RX_BUF_SIZE);
    } else {
        std::cout << "Failed to read from serial port: " << ec.message()
                  << std::endl;
    }
    this->reading = false;
}

void RobotSerial::startReading() {
    serial.async_read_some(asio::buffer(this->rxBuf, SERIAL_RX_BUF_SIZE),
                           [this](const asio::error_code &ec, size_t len) {
                               this->onRead(ec, len);
                           });
    this->reading = true;
}

void RobotSerial::run(bool checksum) {
    if (io.stopped()) {
        io.reset();
    }
    sendCurrentQueue(checksum);
    io.poll();

    if (serial.is_open() && !reading) {
        startReading();
    }
}

bool RobotSerial::isConnected() { return serial.is_open(); }

void RobotSerial::enqueueMessage(SerialPacket *mess) {
    outgoingQueue.push(*mess);
}

void RobotSerial::addChecksum(SerialPacket *packet) {
    uint8_t *ptr = packet->packet;
    uint16_t checksum = fletcher16(ptr, SERIAL_MES_LEN - 2);
    packet->portions.checksumHigh = (checksum >> 8) & 0xff;
    packet->portions.checksumLow = checksum & 0xff;
}

uint16_t RobotSerial::fletcher16(const uint8_t *data, size_t len) {
    uint32_t c0, c1;

    /*  Found by solving for c1 overflow: */
    /* n > 0 and n * (n+1) / 2 * (2^8-1) < (2^32-1). */
    for (c0 = c1 = 0; len > 0;) {
        size_t blocklen = len;
        if (blocklen > 5802) {
            blocklen = 5802;
        }
        len -= blocklen;
        do {
            c0 = c0 + *data++;
            c1 = c1 + c0;
        } while (--blocklen);
        c0 = c0 % 255;
        c1 = c1 % 255;
    }
    return (c1 << 8 | c0);
}

std::string SerialPacket::GetLogMessage() {
    std::string msg;

    for (size_t i = 0; i < SERIAL_MES_DATA_LEN; i++) {
        uint8_t c = portions.data[i];

        if (c != 0) {
            msg.push_back((char)c);
        } else {
            break;
        }
    }

    return msg;
}

SerialPacketType SerialPacket::GetType() {
    return (SerialPacketType)portions.messageType;
}
