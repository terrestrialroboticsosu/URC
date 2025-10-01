#pragma once
#include "robotArm.h"
#include <vector>
#include <random>

/**
 * @class ArmTester
 * @brief A helper class to test the RobotArm's IK solver with random points.
 */
class ArmTester {
public:
    /**
     * @brief Constructs an ArmTester.
     * @param arm A reference to the RobotArm object to be tested.
     */
    ArmTester(RobotArm& arm);

    /**
     * @brief Runs the IK test on 1000 randomly generated points.
     *
     * This method will call moveToPose() for each point and report the
     * overall success rate to the console.
     */
    void runTest() const;

private:
    /**
     * @brief Generates a set of random target poses within a sphere.
     */
    void generateTestPoints();

    RobotArm& robot_arm; // A reference to the arm we are testing
    std::vector<KDL::Frame> test_points; // Stores the generated target poses

    // Random number generation tools
    std::mt19937 random_engine;
};