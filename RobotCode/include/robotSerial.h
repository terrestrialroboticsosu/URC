#pragma once

#include "asio.hpp"
#include <queue>
#include <cstdint>

#define SERIAL_MES_LEN 13
#define SERIAL_RX_BUF_SIZE 128
#define SERIAL_MES_DATA_LEN 8

#define SERIAL_MSG_TYPE_INTAKE_POS_TLM 0x41
#define SERIAL_MSG_TYPE_LOG 0x61

enum SerialPacketType {
    SERIAL_PACKET_INTAKE_POS = 0x41,
    SERIAL_PACKET_LOG = 0x61,
};

union SerialPacket
{
    uint8_t packet[SERIAL_MES_LEN];
    struct {
        private: const uint16_t syncBytes = 0xBEEF; // tf is this??
        public:
        uint8_t messageType = 0x00;
        uint8_t data[SERIAL_MES_DATA_LEN] = {0};
        uint8_t checksumHigh;
        uint8_t checksumLow;
    } portions;

    SerialPacketType GetType();

    int GetIntakePos();
    std::string GetLogMessage();
};

class RobotSerial
{

public:
    RobotSerial(std::string port, unsigned int baud_rate);
    bool readNextMessage(SerialPacket *packet);
    void sendHeartbeat(); // TODO: Probably make this method virtual and let the subclasses have their own version
    bool isConnected();

    int sendCurrentQueue();
    void run();
    void sendBytesHandler(const asio::error_code& error, std::size_t bytes_transferred);
    void enqueueMessage(SerialPacket * mess);
    void addChecksum(SerialPacket * packet);
    void startReading();
    void onRead(const asio::error_code& ec, size_t len);
    uint16_t fletcher16(const uint8_t *data, size_t len);

private:
    asio::io_service io;
    asio::serial_port serial;

    std::queue<SerialPacket> outgoingQueue;
    std::queue<uint8_t> recvData;
    uint8_t outgoingBytes[13];
    uint8_t rxBuf[SERIAL_RX_BUF_SIZE];
    int positonOfNextOutgoingByte = 0;
    bool byteQueueFull= false;
    bool serialTransmit = false;
    bool reading = false;

};

