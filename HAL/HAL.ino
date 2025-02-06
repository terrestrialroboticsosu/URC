#include <Servo.h>
#include <Adafruit_MCP2515.h>
#include "AS5600.h"
#include <Wire.h>

extern "C" {
#include "pico/bootrom.h"
}

// -- constants --
#define MESSAGE_TYPE_INTAKE_POSITION 0x41
#define MESSAGE_TYPE_MOTOR_CMD_ACK 0x60
#define MESSAGE_TYPE_LOG_MESSAGE 0x61
#define MESSAGE_TYPE_HEARTBEAT 0x01

#define CAN_ID_MANUFACTURER_REV 5
#define CAN_ID_DEVICE_TYPE_MOTOR_CONTROLLER 2

#define SPARK_MAX_API_CLASS_DUTY_CYCLE 0
#define SPARK_MAX_API_CLASS_HEARTBEAT 9

#define SPARK_MAX_API_INDEX_DUTY_CYCLE 2
#define SPARK_MAX_API_INDEX_HEARTBEAT 2

#define SPARK_MAX_ID_MAX 31

#define ROBOT_TIMEOUT_MS 1000

#define SPARK_MAX_HEARTBEAT_TIME_MS 5

#define SERIAL_PACKET_SIZE 13
#define SERIAL_PAYLOAD_SIZE 8

#define HEARTBEAT_LOG_RATE_MS 1000

#define TELEMETRY_RATE_MS 200

// Set CAN bus baud rate
#define CAN_BAUDRATE (1000000)

typedef enum error_code {
  ERROR_CODE_MCP2515_INIT_FAILURE = 1,
  ERROR_CODE_BAD_CHECKSUM = 2,
  ERROR_CODE_BAD_MSG_TYPE = 3,
  ERROR_CODE_ENCODER_INIT_FAILURE = 4,
  ERROR_CODE_ENCODER_MAG_FAILURE = 5,
} error_code_t;

Adafruit_MCP2515 mcp(PIN_CAN_CS);

const int FRONT_LEFT_MOTOR_PIN = 25;
const int BACK_LEFT_MOTOR_PIN = 9;
const int FRONT_RIGHT_MOTOR_PIN = 27;
const int BACK_RIGHT_MOTOR_PIN = 29;
const int DEPLOY_MOTOR_PIN = 11;

const int INTAKE_MOTOR_CAN_ID = 9;
const int DUMP_MOTOR_CAN_ID = 10;

Servo frontLeftMotor;
Servo backLeftMotor;
Servo frontRightMotor;
Servo backRightMotor;
Servo deployMotor;

const int PACKET_SIZE = 13;
const int BUFFER_SIZE = 1;

uint8_t packet[PACKET_SIZE];
int currentIndex = 0;

const int INDEX_NO_PACKET = -1;

bool SparkMaxEnabled[SPARK_MAX_ID_MAX];
float intakeSpeed = 0.0;
float dumpSpeed = 0.0;

unsigned long lastPacket;
unsigned long lastSparkMaxPacket;
int sparkMaxPacketCount = 0;

unsigned long lastLogHeartbeat = 0;
unsigned long lastTelemetry;

float intakeAngle = 0;

AMS_5600 ams5600;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  while(!Serial) delay(10);

  SendLogMessage("BOOT");
  if (!mcp.begin(CAN_BAUDRATE)) {
    SendError(ERROR_CODE_MCP2515_INIT_FAILURE);
    while(1) delay(10);
  }
  SendLogMessage("CAN OK");

  frontLeftMotor.attach(FRONT_LEFT_MOTOR_PIN);
  backLeftMotor.attach(BACK_LEFT_MOTOR_PIN);
  frontRightMotor.attach(FRONT_RIGHT_MOTOR_PIN);
  backRightMotor.attach(BACK_RIGHT_MOTOR_PIN);
  deployMotor.attach(DEPLOY_MOTOR_PIN);

  for(int i = 0; i < SPARK_MAX_ID_MAX; i++) {
    SetSparkMaxEnabled(i, false);
  }

  if(ams5600.isConnected()) {
    if(ams5600.detectMagnet()) {
      SendLogMessage("MAG OK");
    } else {
      SendError(ERROR_CODE_ENCODER_MAG_FAILURE);
    }
  } else {
    SendError(ERROR_CODE_ENCODER_INIT_FAILURE);
  }

  SendLogMessage("INIT OK");
}

void loop() {
  updateSerial();
  unsigned long currentTime = millis();
  
  // -- check timings for each packet --
  if(currentTime - lastPacket > ROBOT_TIMEOUT_MS) {
    intakeSpeed = 0.0;
    SendSparkMaxSpeed(INTAKE_MOTOR_CAN_ID, 0.0);
    delay(5);
    SendSparkMaxSpeed(DUMP_MOTOR_CAN_ID, 0.0);
    SetDriveMotors(0, 0, 0, 0);
    SetDeployMotor(0);
    
    for(int i = 0; i < SPARK_MAX_ID_MAX; i++) {
      SetSparkMaxEnabled(i, false);
    }
  }

  if(currentTime - lastSparkMaxPacket > SPARK_MAX_HEARTBEAT_TIME_MS) {
    if(sparkMaxPacketCount == 0) {
      SendSparkMaxHeartbeat();
    } else if (sparkMaxPacketCount == 1) {
      SendSparkMaxSpeed(DUMP_MOTOR_CAN_ID, dumpSpeed);
    } else if (sparkMaxPacketCount == 2) {
      SendSparkMaxSpeed(INTAKE_MOTOR_CAN_ID, intakeSpeed);
    }

    sparkMaxPacketCount++;
    if(sparkMaxPacketCount > 2) {
      sparkMaxPacketCount = 0;
    }

    lastSparkMaxPacket = currentTime;
  }

  if(currentTime - lastLogHeartbeat > HEARTBEAT_LOG_RATE_MS) {
    SendLogMessage("UPDATE");
    lastLogHeartbeat = currentTime;
  }

  if(currentTime - lastTelemetry > TELEMETRY_RATE_MS) {
    ReadCurrentAngle();
    SendIntakePosition(); 
    lastTelemetry = currentTime;
  }
}

void updateSerial() {
  if (!Serial.available()) {
    return;
  }
  uint8_t incomingByte = Serial.read();
  if (currentIndex == INDEX_NO_PACKET) {
    //check if it is the sync byte(0xBE)
    if (incomingByte == 0xBE) {
      currentIndex = 0;
      packet[currentIndex++] = incomingByte;
    }
  } else {
    if (currentIndex == 1) {
      if (incomingByte != 0xEF) {
        currentIndex = INDEX_NO_PACKET;
        return;
      }
    }
    packet[currentIndex++] = incomingByte;
    if (currentIndex >= PACKET_SIZE) {
      processPacket();
      currentIndex = 0;
    }
  }
}

void processPacket()
{
  // Ensure matching checksum, else return from function call. 
  if (!verifyChecksum)
  {
    SendError(ERROR_CODE_BAD_CHECKSUM);
    return;
  }

  lastPacket = millis();

  // Packet has a valid checksum, now process
  switch (packet[2]) {
    case 0x81:
      HandleSetDriverMotorPacket();
      break;
    case 0x83:
      HandleSetIntakeSpeedPacket();
      break;
    case 0x84: 
      HandleSetDumpSpeedPacket();
      break;
    case 0x82:
      HandleSetDeployMotorPacket();
      break;
    case 0xA0:
      SendLogMessage("BOOTLOAD");
      reset_usb_boot(1 << LED_BUILTIN, 0);
      break;
    case MESSAGE_TYPE_HEARTBEAT:
      break;
    default:
      SendError(ERROR_CODE_BAD_MSG_TYPE);
  }
}

void HandleSetDumpSpeedPacket() {
  int8_t speed = packet[3];

  dumpSpeed = ((float) speed) / 100.0;
  SendMotorCommandAck();
  SetSparkMaxEnabled(DUMP_MOTOR_CAN_ID, true);
}

void HandleSetIntakeSpeedPacket() {
  uint8_t speed = packet[3];

  intakeSpeed = -((float) speed) / 100.0;
  SendMotorCommandAck();
  SetSparkMaxEnabled(INTAKE_MOTOR_CAN_ID, true);
}

void HandleSetDriverMotorPacket()
{
  SendMotorCommandAck();
  SetDriveMotors((int8_t)packet[3], (int8_t)packet[4], (int8_t)packet[5], (int8_t)packet[6]);
}

void SetDriveMotors(int8_t frontLeft, int8_t frontRight, int8_t backLeft, int8_t backRight) {
  frontLeftMotor.writeMicroseconds(map(frontLeft, -100, 100, 1000, 2000));
  frontRightMotor.writeMicroseconds(map(-frontRight, -100, 100, 1000, 2000));
  backLeftMotor.writeMicroseconds(map(backLeft, -100, 100, 1000, 2000));
  backRightMotor.writeMicroseconds(map(-backRight, -100, 100, 1000, 2000));
}

void HandleSetDeployMotorPacket() {
  SendMotorCommandAck();
  SetDeployMotor((int8_t) packet[3]);
}

void SetDeployMotor(int8_t speed) {
  deployMotor.writeMicroseconds(map(-speed, -100, 100, 1000, 2000));
}

inline bool verifyChecksum()
{
   uint16_t correctChecksum = fletcher16(packet, PACKET_SIZE-2);
  uint8_t lowChecksumByte = correctChecksum & 0xff;
  uint8_t highChecksumByte = (correctChecksum>> 8) & 0xff;

  return  lowChecksumByte == packet[12] && highChecksumByte == packet[11];
}

uint16_t fletcher16(const uint8_t *data, size_t len) {
	uint32_t c0, c1;

	/*  Found by solving for c1 overflow: */
	/* n > 0 and n * (n+1) / 2 * (2^8-1) < (2^32-1). */
	for (c0 = c1 = 0; len > 0; ) {
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

typedef union
{
  float number;
  uint8_t bytes[4];
} float_union_t;

uint32_t CreateSparkMaxID(int api_class, int api_index, int device_id) {
  uint32_t device_id_bits = device_id & 0x3F; // Bits [0-5] are the device ID
  uint32_t api_index_bits = (api_index & 0x0F) << 6; // Bits [6-9] are the API index
  uint32_t api_class_bits = (api_class & 0x3F) << 10; // Bits [10-15] are the API class
  uint32_t manufacturer_id_bits = (CAN_ID_MANUFACTURER_REV & 0xFF) << 16;
  uint32_t device_type_bits  = (CAN_ID_DEVICE_TYPE_MOTOR_CONTROLLER & 0x1F) << 24;
  uint32_t id = device_id_bits | api_index_bits | api_class_bits | manufacturer_id_bits | device_type_bits;

  return id;
}

void SendSparkMaxSpeed(int motorID, float speed) {
  float_union_t speed_bytes;
  speed_bytes.number = speed;

  uint32_t packetID = CreateSparkMaxID(SPARK_MAX_API_CLASS_DUTY_CYCLE, SPARK_MAX_API_INDEX_DUTY_CYCLE, motorID);
  mcp.beginExtendedPacket(packetID);
  mcp.write(speed_bytes.bytes[0]);
  mcp.write(speed_bytes.bytes[1]);
  mcp.write(speed_bytes.bytes[2]);
  mcp.write(speed_bytes.bytes[3]);
  mcp.write(0x00);
  mcp.write(0x00);
  mcp.write(0x00);
  mcp.write(0x00);
  mcp.endPacket();
}

void SendSparkMaxHeartbeat() {
  uint32_t packetID = CreateSparkMaxID(SPARK_MAX_API_CLASS_HEARTBEAT, SPARK_MAX_API_INDEX_HEARTBEAT, 0); 
  uint8_t bytes[8] = {0, 0};

  for(int i = 0; i < SPARK_MAX_ID_MAX; i++) {
    int byte = i / 8;
    int bit = i % 8;

    if(SparkMaxEnabled[i]) {
      bytes[byte] |= (0x01 << bit);
    }
  }

  mcp.beginExtendedPacket(packetID);
  for(int i = 0; i < 8; i++) {
    mcp.write(bytes[i]);
  }
  mcp.endPacket();
}

void SetSparkMaxEnabled(int device_id, bool enabled) {
  SparkMaxEnabled[device_id] = enabled;
}

void SendPacket(uint8_t message_type, uint8_t data[SERIAL_PAYLOAD_SIZE]) {
  // Fill packet with message type / data
  uint8_t buf[SERIAL_PACKET_SIZE] = {0};
  buf[0] = 0xBE;
  buf[1] = 0xEF;
  buf[2] = message_type;
  memcpy(buf + 3, data, SERIAL_PAYLOAD_SIZE);
  
  // Add checksum
  uint16_t checksum = fletcher16(buf, SERIAL_PAYLOAD_SIZE + 3);
  buf[SERIAL_PACKET_SIZE - 2] = checksum & 0xFF;
  buf[SERIAL_PACKET_SIZE - 1] = (checksum >> 8) & 0xFF;

  // Send packet
  Serial.write(buf, SERIAL_PACKET_SIZE);
}

void SendMotorCommandAck() {
  uint8_t data[SERIAL_PAYLOAD_SIZE] = {0};
  SendPacket(MESSAGE_TYPE_MOTOR_CMD_ACK, data);
}

void SendLogMessage(const char *msg) { 
  char data[SERIAL_PAYLOAD_SIZE] = {0};
  strncpy(data, msg, SERIAL_PAYLOAD_SIZE);
  SendPacket(MESSAGE_TYPE_LOG_MESSAGE, (uint8_t*) data);
}

void SendError(error_code_t code) {
  char msg[SERIAL_PAYLOAD_SIZE] = {0};
  snprintf(msg, SERIAL_PAYLOAD_SIZE, "ERR %d", code);

  SendLogMessage(msg);
}

void ReadCurrentAngle()
{
  int rawAngle;

  if(ams5600.tryGetRawAngle(&rawAngle)) {
    intakeAngle = rawAngle * 0.087;
  }
}

void SendIntakePosition() {
  int angleInt = intakeAngle * 100;
  uint8_t data[SERIAL_PAYLOAD_SIZE] = {0};
  data[0] = angleInt & 0xFF;
  data[1] = (angleInt >> 8) & 0xFF;

  SendPacket(MESSAGE_TYPE_INTAKE_POSITION, data);
}
