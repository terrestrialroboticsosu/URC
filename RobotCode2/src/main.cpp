#include <iostream>
#include <limits>
#include "robotArm.h"
#include <kdl/frames_io.hpp>

#include "armTester.h"

void handleInvalidInput() {
    std::cout << "Invalid input. Please enter a valid number." << std::endl;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

int main() {
    std::cout << "Initializing Robot Arm..." << std::endl;
    RobotArm arm; // The arm now initializes itself to the zero position.

    while (true) {
        std::cout << "\n---------------------------------" << std::endl;
        std::cout << "Current End-Effector Position: " << arm.getEndEffectorPose().p << std::endl;
        arm.printSegmentPositions();

        std::cout << "Select Input Mode:" << std::endl;
        std::cout << "  0: Set target by (x, y, z) coordinates (IK)" << std::endl;
        std::cout << "  1: Set individual joint angle" << std::endl;
        std::cout << "  2: Run joint angle test" << std::endl;
        std::cout << "  Enter any other key to quit." << std::endl;
        std::cout << "Mode: ";

        int inputMode;
        std::cin >> inputMode;

        if (!std::cin) {
            std::cout << "Exiting program." << std::endl;
            break;
        }

        if (inputMode == 0) {
            double x, y, z;
            std::cout << "Enter target coordinates:" << std::endl;
            std::cout << "x: ";
            if (!(std::cin >> x)) { handleInvalidInput(); continue; }
            std::cout << "y: ";
            if (!(std::cin >> y)) { handleInvalidInput(); continue; }
            std::cout << "z: ";
            if (!(std::cin >> z)) { handleInvalidInput(); continue; }

            KDL::Frame target_pose = KDL::Frame(KDL::Vector(x, y, z));
            std::cout << "\nCalculating IK for target: " << target_pose.p << std::endl;

            if (arm.moveToPose(target_pose)) {
                std::cout << "Solution found! New joint angles (radians):" << std::endl;
                for (unsigned int i = 0; i < arm.getNumJoints(); i++) {
                    std::cout << "  Joint " << i << ": " << arm.getJointAngle(i) << std::endl;
                }
                //arm.toDesmosVector();
            } else {
                std::cout << "IK solver could not find a solution." << std::endl;
            }

        } else if (inputMode == 1) {
            unsigned int index;
            double angle;
            std::cout << "Enter joint index and desired angle (in radians):" << std::endl;
            std::cout << "index (0-" << arm.getNumJoints() - 1 << "): ";
            if (!(std::cin >> index)) { handleInvalidInput(); continue; }
            std::cout << "angle: ";
            if (!(std::cin >> angle)) { handleInvalidInput(); continue; }

            if (arm.setJointAngle(index, angle)) {
                 std::cout << "Joint angle updated." << std::endl;
                 //arm.toDesmosVector();
            }
        } else if (inputMode == 2) {
            ArmTester tester(arm);
            tester.runTest();
        } else {
            std::cout << "Exiting program." << std::endl;
            break;
        }
    }

    std::cout << "\nFinal arm end-effector position: " << arm.getEndEffectorPose().p << std::endl;
    return 0;
}