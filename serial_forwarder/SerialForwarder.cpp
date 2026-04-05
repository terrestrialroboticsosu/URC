#include "SerialForwarder.h"
#include <iostream>
#include <cstring>
#include <asio.hpp>

#define SERIAL_PACKET_LEN 13
#define SERIAL_DATA_LEN 8

SerialForwarder::SerialForwarder(const std::string& radioPort, const std::string& halPort, unsigned int baud)
    : radioSerial(io), halSerial(io), sawFirstPacket(false), reading(false) {
    try {
        radioSerial.open(radioPort);
        radioSerial.set_option(asio::serial_port_base::baud_rate(baud));
        std::cout << "Opened radio port: " << radioPort << std::endl;

        halSerial.open(halPort);
        halSerial.set_option(asio::serial_port_base::baud_rate(baud));
        std::cout << "Opened HAL port: " << halPort << std::endl;
    } catch (asio::system_error &e) {
        std::cerr << "Failed to open serial port: " << e.what() << std::endl;
    }
}

bool SerialForwarder::isConnected() const {
    return radioSerial.is_open() && halSerial.is_open();
}

void SerialForwarder::run() {
    if (!reading) startReadingRadio();
    io.run();
}

// Start asynchronous reading
void SerialForwarder::startReadingRadio() {
    radioSerial.async_read_some(asio::buffer(rxBuf, sizeof(rxBuf)),
                                [this](const asio::error_code &ec, std::size_t len) { onReadRadio(ec, len); });
    reading = true;
}

// Async read handler
void SerialForwarder::onReadRadio(const asio::error_code &ec, std::size_t len) {
    if (!ec) {
        for (size_t i = 0; i < len; ++i) {
            recvData.push_back(rxBuf[i]);
            std::cout << "[RADIO RX] 0x" << std::hex << (int)rxBuf[i] << std::dec
                      << " (" << (int)rxBuf[i] << ")" << std::endl;
        }

        parsePackets();
    } else {
        std::cerr << "Radio read error: " << ec.message() << std::endl;
    }

    // Continue reading
    startReadingRadio();
}

// Parse full packets
void SerialForwarder::parsePackets() {
    while (recvData.size() >= SERIAL_PACKET_LEN) {
        // Look for sync bytes
        size_t pos = 0;
        while (pos + 1 < recvData.size() && !(recvData[pos] == 0xBE && recvData[pos + 1] == 0xEF))
            ++pos;

        // Remove noise before sync
        for (size_t i = 0; i < pos; ++i) recvData.pop_front();

        if (recvData.size() < SERIAL_PACKET_LEN) break;

        // Remove sync bytes
        recvData.pop_front();
        recvData.pop_front();

        if (!sawFirstPacket) {
            // Skip first packet entirely
            sawFirstPacket = true;
            for (int i = 0; i < SERIAL_PACKET_LEN - 2; ++i) recvData.pop_front();
            std::cout << "[INFO] First packet skipped" << std::endl;
            continue;
        }

        // Second packet — process normally
        sawFirstPacket = false;

        SerialPacket packet;
        packet.messageType = recvData.front(); recvData.pop_front();
        for (int i = 0; i < SERIAL_DATA_LEN; ++i) {
            packet.data[i] = recvData.front(); 
            recvData.pop_front();
        }
        packet.checksumHigh = recvData.front(); recvData.pop_front();
        packet.checksumLow  = recvData.front(); recvData.pop_front();

        // Print packet
        std::cout << "=== GOOD PACKET PICKED UP ===" << std::endl;
        std::cout << "Type: " << (int)packet.messageType << " Data: ";
        for (int i = 0; i < SERIAL_DATA_LEN; ++i) std::cout << (int)packet.data[i] << " ";
        std::cout << "Checksum: " << (int)packet.checksumHigh << " " << (int)packet.checksumLow << std::endl;
        std::cout << "============================" << std::endl;

        sendToHAL(packet);
    }
}

// Forward packet to HAL
void SerialForwarder::sendToHAL(const SerialPacket &packet) {
    uint8_t buffer[SERIAL_PACKET_LEN];
    buffer[0] = 0xBE;
    buffer[1] = 0xEF;
    buffer[2] = packet.messageType;
    for (int i = 0; i < SERIAL_DATA_LEN; ++i) buffer[3 + i] = packet.data[i];
    buffer[11] = packet.checksumHigh;
    buffer[12] = packet.checksumLow;

    asio::write(halSerial, asio::buffer(buffer, SERIAL_PACKET_LEN));

    std::cout << "[HAL TX] Type: " << (int)packet.messageType << " Data: ";
    for (int i = 0; i < SERIAL_DATA_LEN; ++i) std::cout << (int)packet.data[i] << " ";
    std::cout << std::endl;
}
