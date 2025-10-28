#include "armTester.h"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <kdl/frames_io.hpp>

ArmTester::ArmTester(RobotArm& arm) : robot_arm(arm) {
    // Seed the random number generator
    std::random_device rd;
    random_engine.seed(rd());

    generateTestPoints();
}

void ArmTester::generateTestPoints() {
    constexpr int num_points = 1000;
    const KDL::Vector center(0.0, 0.0, 0.15);

    test_points.reserve(num_points);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    for (int i = 0; i < num_points; ++i) {
        constexpr double radius = 1;
        const double theta = 2.0 * M_PI * dist(random_engine);
        const double phi = acos(1.0 - 2.0 * dist(random_engine));
        const double r = radius * cbrt(dist(random_engine));

        const double x = abs(r * sin(phi) * cos(theta));
        const double y = abs(r * sin(phi) * sin(theta));
        const double z = r * cos(phi);

        // Create the KDL Vector and add the center offset
        KDL::Vector point_vec(center.x() + x, center.y() + y, center.z() + z);
        // Create the target Frame and add it to our list
        test_points.emplace_back(point_vec);
    }
    std::cout << num_points << " test points generated." << std::endl;
}

void ArmTester::runTest() const {
    std::cout << "\n--- Starting IK Solver Test ---" << std::endl;

    int success_count = 0;
    int far = 0;
    int close = 0;

    for (const auto& target_pose : test_points) {
        KDL::Vector position = target_pose.p;
        if (robot_arm.moveToPose(target_pose)) {
            success_count++;
            robot_arm.moveToDefaultPose();
            std::cout << "(" << std::fixed << std::setprecision(4)
              << position.x() << ","
              << position.y() << ","
              << position.z() << ")" << std::endl;
        } else {

            if ((pow(position.x(),2) + pow(position.y(),2) + pow(position.z()+0.15,2)) > 1) {
                far++;
            } else if ((pow(position.x(),2) + pow(position.y(),2)) < pow(0.5,2) && position.z() < 0) {
                close++;
            } else {

            }
        }
    }

    // --- Report Results ---
    double success_rate = (static_cast<double>(success_count) / test_points.size()) * 100.0;
    std::cout << "\n--- Test Complete ---" << std::endl;
    std::cout << "IK Solver Success Rate: " << std::fixed << std::setprecision(2)
              << success_rate << "% (" << success_count << "/" << test_points.size() << ")"
              << std::endl;
    std::cout << "Points farther than 1: " << far << std::endl;
    std::cout << "Points closer than 0.5 to z axis: " << close << std::endl;

    int success_count2 = success_count + far + close;
    double success_rate2 = (static_cast<double>(success_count2) / test_points.size()) * 100.0;
    std::cout << "IK Solver Success Rate (excluding outliers): " << std::fixed << std::setprecision(2)
              << success_rate2 << "% (" << success_count2 << "/" << test_points.size() << ")"
              << std::endl;
}
