#include "robotControl.h"
#include "robotState.h"
#include "util.h"
#include <iostream>

#define HEARTBEAT_RATE_MS 1000
#define MOTOR_UPDATE_RATE_MS 1000
#define INTAKE_SPEED 100
#define DUMP_SPEED 100
#define CRAWL_SPEED 40
#define DEPLOY_LOWER_SPEED -15
#define DEPLOY_RAISE_SPEED 65
#define TRIGGER_DEADZONE 10 

void RobotControl::sendStateToRP2040(RobotActuation *rp2040) {
  uint64_t currentTime = getUnixTimeMs();

  if (currentTime - this->lastHeartbeat > HEARTBEAT_RATE_MS) {
    std::cout << "Send Heartbeat to RP2040" << std::endl;
    rp2040->sendHeartbeat();
    lastHeartbeat = currentTime;
  }

  // bool frontLeftChange = currentState.getDriveFrontLeft() != lastStateSent.getDriveFrontLeft();
  // bool frontRightChange = currentState.getDriveFrontLeft() != lastStateSent.getDriveFrontLeft();
  // bool backLeftChange = currentState.getDriveFrontLeft() != lastStateSent.getDriveFrontLeft();
  // bool backRightChange = currentState.getDriveFrontLeft() != lastStateSent.getDriveFrontLeft();

  // if (frontLeftChange || frontRightChange || backLeftChange || backRightChange ||
  //     currentTime - this->lastDriveCmd > MOTOR_UPDATE_RATE_MS) {
  //   std::cout << "Send Drive to RP2040 (FL=" << +currentState.getDriveFrontLeft() << ", FR=" << +currentState.getDriveFrontLeft()
  //             << ", BL=" << +currentState.getDriveFrontLeft() << ", BR=" << +currentState.getDriveFrontLeft() << ")" << std::endl;
  //   rp2040->sendDriveMotors(currentState.getDriveFrontLeft(), currentState.getDriveFrontRight(), currentState.getDriveBackLeft(), currentState.getDriveBackRight());
  //   lastDriveCmd = currentTime;
  // }

  // if(currentState.getIntake() != lastStateSent.getIntake() || currentTime - lastIntakeCmd > MOTOR_UPDATE_RATE_MS) {
  //   std::cout << "Send Intake to RP2040: " << +currentState.getIntake() << std::endl; 
  //   rp2040->sendIntakeMotor(currentState.getIntake());
  //   lastIntakeCmd = currentTime;
  // }

  // if(currentState.getDump() != lastStateSent.getDump() || currentTime - lastDumpCmd > MOTOR_UPDATE_RATE_MS) {
  //   std::cout << "Send Dump to RP2040: " << +currentState.getDump() << std::endl; 
  //   rp2040->sendDumpMotor(currentState.getDump());
  //   lastDumpCmd = currentTime;
  // }

  // if(currentState.getDeploy() != lastStateSent.getDeploy() || currentTime - lastDeployCmd > MOTOR_UPDATE_RATE_MS) {
  //   std::cout << "Send Deploy to RP2040: " << +currentState.getDeploy() << std::endl; 
  //   rp2040->sendDeployControl(currentState.getDeploy());
  //   lastDeployCmd = currentTime;
  // }

  // lastStateSent = currentState;
}

void RobotControl::handleGamepadPacket(GamepadPacket packet) {
  // Drive
  // int leftDriveSpeed = packet.getLeftStickY();
  // int rightDriveSpeed = packet.getRightStickY();
  // if(packet.isDpadUp()) {
  //   leftDriveSpeed = CRAWL_SPEED;
  //   rightDriveSpeed = CRAWL_SPEED;
  // } else if (packet.isDpadDown()) {
  //   leftDriveSpeed = -CRAWL_SPEED;
  //   rightDriveSpeed = -CRAWL_SPEED;
  // }
  // currentState.setDrive(leftDriveSpeed, rightDriveSpeed);

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

// int RobotControl::robotRunStateMachine()
// {
//     while(running) {
//         switch (currentState)
//         {
//             case 0:
//                 runStartupState();
//                 break;
//             case 1:
//                 runStopState();
//                 break;
//             case 2:
//                 runTeleopState();
//                 break;
//             case 3:
//                 runAutoState();
//             default:
//                 runPanicState();

//         }
//     }
//     return 1;
// }

// //Heart beat every seond

// int RobotControl::robotStartup()
// {
//     return robotRunStateMachine();
// }

// int RobotControl::runStartupState()
// {
//     return 1;
// }

// int RobotControl::runAutoState()
// {
//     currentState = -1;
//     return 1;
// }

// int RobotControl::runStopState()
// {
//     setDriveValues(0,0,0,0);
//     setIntakeSpeed(0);
//     setDumpSpeed(0);
//     setIntakePosition(trueState.intakeLocation);

//     return 1;
// }

// int RobotControl::runTeleopState()
// {
//     return 1;
// }

// int RobotControl::runPanicState()
// {
//     // Robot is paniced!
//     setDriveValues(0,0,0,0);
//     setIntakeSpeed(0);
//     setDumpSpeed(0);
//     setIntakePosition(trueState.intakeLocation);
//     currentState = -1;

//     return -1;
// }

// int RobotControl::setDumpSpeed(int8_t val)
// {
//     return 0;
// }

// int RobotControl::setIntakePosition(int8_t val)
// {
//     return 0;
// }

// int RobotControl::setIntakeSpeed(int8_t val)
// {
//     return 0;
// }

// int RobotControl::setDriveValues(int8_t flVal, int8_t frVal, int8_t blVal, int8_t brVal)
// {
//     return 0;
// }
