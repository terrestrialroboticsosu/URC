#include <Servo.h>

// ================= SERVOS =================
Servo shoulder;
Servo elbow;
Servo wrist1;
Servo wrist2;
Servo driveLeft;
Servo driveRight;
Servo turret;

// Pins
const int PIN_SHOULDER    = 25;
const int PIN_ELBOW       = 11;
const int PIN_WRIST1      = 27;
const int PIN_WRIST2      = 15;
const int PIN_DRIVE_LEFT  = 3;
const int PIN_DRIVE_RIGHT = 9;
const int PIN_TURRET      = 26;

// ================= SPEED LIMITS =================
const int STOP_US = 1500;

const int SHOULDER_FWD = 1750;
const int SHOULDER_REV = 1250;

const int ELBOW_FWD = 1700;
const int ELBOW_REV = 1300;

const int WRIST_FWD = 1525;
const int WRIST_REV = 1475;

const int DRIVE_FWD = 1600;
const int DRIVE_REV = 1400;

const int TURRET_FWD = 1600;
const int TURRET_REV = 1400;

// ================= STEPPER =================
const int DIR_PIN       = 5;
const int STEP_PIN      = 2;
const int STEP_DELAY_US = 1000;

bool stepperEnabled = false;
bool stepperForward = true;

// ================= PACKET =================
// [0xBE, 0xEF, left, right, shoulder, elbow, wristB, wristT, turret, stepper, 0, 0]
#define PACKET_SIZE 13

uint8_t packet[PACKET_SIZE];
int packetIndex = -1;

// ================= HELPERS =================

int mapClamped(int val, int fwdLimit, int revLimit) {
  int centered = val - 100;
  int us;

  if (centered >= 0) {
    us = STOP_US + (centered * (fwdLimit - STOP_US) / 100);
  } else {
    us = STOP_US + (centered * (STOP_US - revLimit) / 100);
  }

  return max(revLimit, min(fwdLimit, us));
}

void stopAll() {
  driveLeft.writeMicroseconds(STOP_US);
  driveRight.writeMicroseconds(STOP_US);
  shoulder.writeMicroseconds(STOP_US);
  elbow.writeMicroseconds(STOP_US);
  wrist1.writeMicroseconds(STOP_US);
  wrist2.writeMicroseconds(STOP_US);
  turret.writeMicroseconds(STOP_US);
}

void stepOnce() {
  digitalWrite(DIR_PIN, stepperForward ? HIGH : LOW);
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY_US);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY_US);
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  shoulder.attach(PIN_SHOULDER);
  elbow.attach(PIN_ELBOW);
  wrist1.attach(PIN_WRIST1);
  wrist2.attach(PIN_WRIST2);
  driveLeft.attach(PIN_DRIVE_LEFT);
  driveRight.attach(PIN_DRIVE_RIGHT);
  turret.attach(PIN_TURRET);

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);

  stopAll();
}

// ================= PROCESS PACKET =================
void processPacket() {
  uint8_t left      = packet[2];
  uint8_t right     = packet[3];
  uint8_t shoulderV = packet[4];
  uint8_t elbowV    = packet[5];
  uint8_t wristB    = packet[6];
  uint8_t wristT    = packet[7];
  uint8_t turretV   = packet[8];
  uint8_t stepper   = packet[9];

  // ===== DRIVE =====
  driveLeft.writeMicroseconds(mapClamped(200 - left, DRIVE_FWD, DRIVE_REV));
  driveRight.writeMicroseconds(mapClamped(right,     DRIVE_FWD, DRIVE_REV));

  // ===== ARM =====
  shoulder.writeMicroseconds(mapClamped(shoulderV, SHOULDER_FWD, SHOULDER_REV));
  elbow.writeMicroseconds(mapClamped(elbowV,        ELBOW_FWD,    ELBOW_REV));

  // ===== WRIST =====
  int wb = wristB - 100;
  int wt = wristT - 100;

  int w1 = constrain(1500 + (wb + wt) * 2, WRIST_REV, WRIST_FWD);
  int w2 = constrain(1500 + (wb - wt) * 2, WRIST_REV, WRIST_FWD);

  wrist1.writeMicroseconds(w1);
  wrist2.writeMicroseconds(w2);

  // ===== TURRET =====
  turret.writeMicroseconds(mapClamped(turretV, TURRET_FWD, TURRET_REV));

  // ===== STEPPER =====
  if (stepper == 1) {
    stepperForward = true;
    stepperEnabled = true;
  } else if (stepper == 2) {
    stepperForward = false;
    stepperEnabled = true;
  } else {
    stepperEnabled = false;
  }
}

// ================= LOOP =================
void loop() {

  if (stepperEnabled) {
    stepOnce();
  }

  if (Serial.available()) {
    uint8_t b = Serial.read();

    if (packetIndex == -1) {
      if (b == 0xBE) {
        packetIndex = 0;
        packet[packetIndex++] = b;
      }
    } else {
      if (packetIndex == 1 && b != 0xEF) {
        packetIndex = -1;
      } else {
        packet[packetIndex++] = b;

        if (packetIndex >= PACKET_SIZE) {
          processPacket();
          packetIndex = -1;
        }
      }
    }
  }
}