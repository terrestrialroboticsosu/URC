#include <Servo.h>
#include <Wire.h>
#include "AS5600.h"

// ---------------- Constants ----------------
#define SERIAL_PACKET_SIZE 13
#define SERIAL_PAYLOAD_SIZE 8

#define LEFT_MOTOR_PIN 15
#define RIGHT_MOTOR_PIN 27

#define ROBOT_TIMEOUT_MS 3000

// ---------------- Globals ----------------
Servo leftMotor;
Servo rightMotor;

int8_t currentLeftSpeed = 0;
int8_t currentRightSpeed = 0;

uint8_t packet[SERIAL_PACKET_SIZE];
int currentIndex = -1;

unsigned long lastPacket = 0;

AS5600 ams5600;

// ---------------- Setup ----------------
void setup() {
    Serial.begin(115200);
    Wire.begin();
    while (!Serial) delay(10);

    leftMotor.attach(LEFT_MOTOR_PIN);
    rightMotor.attach(RIGHT_MOTOR_PIN);

    // Initialize motors to neutral
    leftMotor.writeMicroseconds(1500);
    rightMotor.writeMicroseconds(1500);

    if (!ams5600.isConnected()) {
        Serial.println("ERR: Encoder not connected");
    }

    lastPacket = millis();
    Serial.println("INIT OK");
}

// ---------------- Main Loop ----------------
void loop() {
    updateSerial();

    // Stop motors if no DS packet received within timeout
    //if (millis() - lastPacket > ROBOT_TIMEOUT_MS) {
    //    SetDriveMotors(0, 0); // neutral
    //}
}

// ---------------- Serial Handling ----------------
void updateSerial() {
    while (Serial.available()) {
        uint8_t incomingByte = Serial.read();
        if (currentIndex == -1) {
            if (incomingByte == 0xBE) {
                currentIndex = 0;
                packet[currentIndex++] = incomingByte;
            }
        } else {
            if (currentIndex == 1 && incomingByte != 0xEF) {
                currentIndex = -1;
                continue;
            }
            packet[currentIndex++] = incomingByte;
            if (currentIndex >= SERIAL_PACKET_SIZE) {
                processPacket();
                currentIndex = -1;
            }
        }
    }
}

void processPacket() {
    lastPacket = millis();

    uint8_t msgType = packet[2];

    switch (msgType) {
        case 0x0B: // DS motor command
            HandleSetDriverMotorPacket();
            break;
        default:
            Serial.print("ERR: Unexpected message type ");
            Serial.println(msgType, HEX);
            break;
    }
}

// ---------------- Motor Control ----------------
void HandleSetDriverMotorPacket() {
    // Map DS packet values: 0x00 = neutral (0), 0x64 = full forward (100)
    int8_t leftSpeed  = convertStickValue(packet[5]);
    int8_t rightSpeed = convertStickValue(packet[7]);

    SetDriveMotors(leftSpeed, rightSpeed);

    // Print packet info
    Serial.println("=== GOOD DS PACKET RECEIVED ===");
    for (int i = 0; i < SERIAL_PACKET_SIZE; i++) {
        if (packet[i] < 16) Serial.print('0');
        Serial.print(packet[i], HEX);
    }
    Serial.println("\n================================");
}

// ---------------- Conversion ----------------
// Converts 0x00 → 0 (neutral), 0x64 → 100 (full forward)
int8_t convertStickValue(uint8_t val) {
    if (val > 0x64) val = 0x64; // cap max
    return (int)val * 100 / 0x64;
}

// ---------------- Motor Output ----------------
void SetDriveMotors(int8_t left, int8_t right) {
    currentLeftSpeed = left;
    currentRightSpeed = right;

    // 0 = neutral (1500 µs), 100 = full forward (2000 µs)
    leftMotor.writeMicroseconds(map(left, 0, 100, 1500, 2000));
    rightMotor.writeMicroseconds(map(right, 0, 100, 1500, 2000));
}
