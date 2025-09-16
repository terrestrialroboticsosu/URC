/**
 * @file robotSerial.h
 * @brief Defines the core structures and class for serial communication.
 *
 * This file contains the low-level definitions for serial packets, including
 * the SerialPacket union and the RobotSerial base class for handling asynchronous
 * serial I/O with Asio.
 */

#pragma once

#include "asio.hpp"
#include <cstdint>
#include <queue>

#define SERIAL_RX_BUF_SIZE 128     /**< Size of the circular buffer for receiving serial data. */
#define SERIAL_MES_DATA_LEN 8      /**< Length of the data payload in a packet. */
#define SERIAL_MES_LEN (SERIAL_MES_DATA_LEN + 5) /**< Total length of a serial packet. */

/**
 * @enum SerialPacketType
 * @brief Defines the different types of messages that can be sent.
 */
enum SerialPacketType {
    PACKET_LOG = 0x61,         /**< A log message packet. */
    PACKET_HEARTBEAT = 0x01,   /**< A heartbeat packet to maintain connection. */
    PACKET_GAMEPAD = 0x02,     /**< A packet containing gamepad/controller data. */
};

/**
 * @union SerialPacket
 * @brief A union to represent a serial message in two ways: as raw bytes or a structured packet.
 *
 * This allows for easy conversion between a raw byte array (for sending/receiving)
 * and a structured format with named fields (for easy data access).
 */
union SerialPacket {
    uint8_t packet[SERIAL_MES_LEN]; /**< View memory as a raw array of bytes. */
    struct {
    private:
        const uint16_t syncBytes = 0xBEEF; /**< Two-byte header (0xBEEF) to identify the start of a packet. */

    public:
        uint8_t messageType = 0x00;        /**< The type of the message (see SerialPacketType). */
        uint8_t data[SERIAL_MES_DATA_LEN] = {0}; /**< The 8-byte data payload. */
        uint8_t checksumHigh;              /**< The high byte of the Fletcher-16 checksum. */
        uint8_t checksumLow;               /**< The low byte of the Fletcher-16 checksum. */
    } portions; /**< View memory as a structured set of fields. */

    /** @return The type of the packet as a SerialPacketType enum. */
    SerialPacketType GetType();

    /**
     * @brief Decodes a log message from the data payload.
     * @return The log message as a std::string.
     */
    std::string GetLogMessage();
};

/**
 * @class RobotSerial
 * @brief A base class for handling asynchronous serial communication.
 *
 * This class uses the Asio library to manage reading from and writing to a serial port
 * without blocking the main program execution.
 */
class RobotSerial {

public:
    /**
     * @brief Construct a new RobotSerial object.
     * @param port The serial port to open (e.g., "/dev/ttyACM0").
     * @param baud_rate The communication speed (e.g., 115200).
     */
    RobotSerial(std::string port, unsigned int baud_rate);

    /**
     * @brief Reads and parses the next complete message from the receive buffer.
     * @param packet A pointer to a SerialPacket to be filled with the message data.
     * @return True if a complete and valid message was read, false otherwise.
     */
    bool readNextMessage(SerialPacket *packet);

    /**
     * @brief Sends a heartbeat packet.
     * @param robotState The current state of the robot.
     * @param rp2040Connected True if the RP2040 is connected.
     */
    void sendHeartbeat(int robotState, bool rp2040Connected);

    /** @return True if the serial port is open and connected, false otherwise. */
    bool isConnected();

    /**
     * @brief Processes the outgoing message queue and sends data over the serial port.
     * @param checksum If true, a checksum will be calculated and added to the packet.
     * @return 0 on success.
     */
    int sendCurrentQueue(bool checksum);

    /**
     * @brief Runs the Asio io_service to process asynchronous events.
     * @param checksum If true, outgoing packets will have a checksum added.
     */
    void run(bool checksum);

    /**
     * @brief Asynchronous handler for when bytes are successfully sent.
     */
    void sendBytesHandler(const asio::error_code &error, std::size_t bytes_transferred);

    /**
     * @brief Adds a message to the outgoing queue to be sent.
     * @param mess A pointer to the SerialPacket to send.
     */
    void enqueueMessage(SerialPacket *mess);

    /**
     * @brief Calculates and adds a Fletcher-16 checksum to the packet.
     * @param packet The packet to add the checksum to.
     */
    void addChecksum(SerialPacket *packet);

    /**
     * @brief Initiates an asynchronous read from the serial port.
     */
    void startReading();

    /**
     * @brief Asynchronous handler for when data is received.
     */
    void onRead(const asio::error_code &ec, size_t len);

    /**
     * @brief Fletcher-16 checksum algorithm.
     * @param data A pointer to the data to be checksummed.
     * @param len The length of the data in bytes.
     * @return The calculated 16-bit checksum.
     */
    uint16_t fletcher16(const uint8_t *data, size_t len);

private:
    asio::io_service io;      /**< The core Asio I/O service. */
    asio::serial_port serial; /**< The Asio serial port object. */

    std::queue<SerialPacket> outgoingQueue; /**< Queue for messages waiting to be sent. */
    std::queue<uint8_t> recvData;          /**< Circular buffer for incoming raw bytes. */
    uint8_t outgoingBytes[13];            /**< Buffer for the packet currently being sent. */
    uint8_t rxBuf[SERIAL_RX_BUF_SIZE];      /**< Raw buffer for asynchronous reads. */
    std::string portName;                 /**< The name of the serial port. */
    int positonOfNextOutgoingByte = 0;   /**< Tracks progress of the current outgoing packet. */
    bool byteQueueFull = false;          /**< True if `outgoingBytes` is full and ready to send. */
    bool serialTransmit = false;         /**< True if an async write is currently in progress. */
    bool reading = false;                /**< True if an async read is currently in progress. */
};