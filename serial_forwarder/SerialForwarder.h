#pragma once
#include <asio.hpp>
#include <deque>
#include <string>
#include <cstdint>

struct SerialPacket {
    uint8_t messageType;
    uint8_t data[8];
    uint8_t checksumHigh;
    uint8_t checksumLow;
};

class SerialForwarder {
public:
    SerialForwarder(const std::string& radioPort, const std::string& halPort, unsigned int baud);
    bool isConnected() const;
    void run();

private:
    asio::io_context io;
    asio::serial_port radioSerial;
    asio::serial_port halSerial;

    std::deque<uint8_t> recvData;
    uint8_t rxBuf[256];

    bool sawFirstPacket;
    bool reading;

    void startReadingRadio();
    void onReadRadio(const asio::error_code &ec, std::size_t len);
    void parsePackets();
    void sendToHAL(const SerialPacket &packet);
};
