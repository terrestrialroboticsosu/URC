#include "robotControl.h"
#include "robotState.h"
#include "util.h"
#include <iostream>

#define HEARTBEAT_RATE_MS 1000
#define MOTOR_UPDATE_RATE_MS 1000
#define TRIGGER_DEADZONE 10 
#define CRAWL_SPEED 40

void RobotControl::sendStateToRP2040(RobotActuation *rp2040) {
  uint64_t currentTime = getUnixTimeMs();

  if (currentTime - this->lastHeartbeat > HEARTBEAT_RATE_MS) {
    std::cout << "Send Heartbeat to RP2040" << std::endl;
    rp2040->sendHeartbeat();
    lastHeartbeat = currentTime;
  }


  // handle robot moving or not. 
  bool leftChange = currentState.getDriveLeft() != lastStateSent.getDriveLeft();
  bool rightChange = currentState.getDriveRight() != lastStateSent.getDriveRight();



  if (leftChange || rightChange || currentTime - this->lastDriveCmd > MOTOR_UPDATE_RATE_MS)
    std::cout << "Send Drive to RP2040 (FL=" << +currentState.getDriveLeft() << ", FR=" << +currentState.getDriveRight()  << std::endl;
    rp2040->sendDriveMotors(currentState.getDriveRight(), currentState.getDriveRight());
    lastDriveCmd = currentTime;


  lastStateSent = currentState;
}

void RobotControl::handleGamepadPacket(GamepadPacket packet) {
  // Drive
  int leftDriveSpeed = packet.getLeftStickY();
  int rightDriveSpeed = packet.getRightStickY();
  if(packet.isDpadUp()) {
     leftDriveSpeed = CRAWL_SPEED;
     rightDriveSpeed = CRAWL_SPEED;
   } else if (packet.isDpadDown()) {
     leftDriveSpeed = -CRAWL_SPEED;
     rightDriveSpeed = -CRAWL_SPEED;
   }
  currentState.setDrive(leftDriveSpeed, rightDriveSpeed);

  // Intake
  /*if(packet.isButtonAPressed()) {
    currentState.setIntake(INTAKE_SPEED);
  } else {
    currentState.setIntake(0);
  }*/

  // Dump
  // if(packet.isButtonBPressed()) {
  //   currentState.setDump(DUMP_SPEED);
  // } else {
  //   currentState.setDump(0);
  // }

  // Deploy
  // int deploySpeed = 0;

  // if(packet.isLeftBumperPressed()) {
  //   deploySpeed = DEPLOY_LOWER_SPEED;
  // } else if(packet.isRightBumperPressed()) {
  //   deploySpeed = DEPLOY_RAISE_SPEED;
  // } else if(packet.getLeftTrigger() > TRIGGER_DEADZONE) { // Lower (loosen)
  //   deploySpeed = -packet.getLeftTrigger() / 2;
  // } else if(packet.getRightTrigger() > TRIGGER_DEADZONE) { // Lift (Tighten)
  //   deploySpeed = packet.getRightTrigger() / 2 * 3;
  // }
  
  // currentState.setDeploy((int8_t) deploySpeed);
}

void RobotControl::handleDsHeartbeatPacket(SerialPacket packet) {
  currentState.setMode((RobotMode)packet.portions.data[0]);
}

void RobotControl::disableRobot() {
  currentState.setMode(ROBOT_MODE_DISABLED);
}

RobotState& RobotControl::getRobotState() {
  return currentState;
}
