/**
 * @file robotArm.cpp
 * @brief Implementation of the RobotArm class.
 */

#include "robotArm.h"
#include <iostream>
#include <iomanip>

RobotArm::RobotArm() {
    // ===================================================================================
    // === DEFINE YOUR 5-AXIS ROBOTIC ARM GEOMETRY HERE ==================================
    // ===================================================================================
    // This is a critical step. You need to replace these placeholder values with the
    // actual measurements and joint types of your arm. The geometry is defined by
    // adding "Segments" to the chain, from the base to the end-effector.
    //
    // Each Segment consists of a Joint and a Frame.
    // - Joint: Defines the type of motion (e.g., RotZ for rotation around Z-axis).
    // - Frame: Defines the static transformation (translation and rotation) from the
    //          end of the previous joint to the start of the current joint.
    //
    // Common Joint Types:
    // - KDL::Joint::RotZ, RotY, RotX: Rotational joints.
    // - KDL::Joint::TransZ, TransY, TransX: Prismatic (linear) joints.
    //
    // Frame(Vector(dx, dy, dz)): dx, dy, dz are the distances in meters.
    //
    // Joint limits also have to be defined
    // - Joint limits are stored as a radian angle from -pi to pi in min_joint_limits and max_joint_limits

    // Adding arm segments
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),KDL::Frame(KDL::Vector(0.0,0.0,0.15))));
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.5))));
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),KDL::Frame(KDL::Vector(0.0,0.0,0.5))));
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ)));
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX)));
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::TransZ),KDL::Frame(KDL::Vector(0.0,0.0,0.15))));

    // Adding joint angles to joint_angles array
    const unsigned int num_joints = arm_chain.getNrOfJoints();
    joint_angles.resize(num_joints);
    joint_angles(0) = 3*M_PI / 4; //base
    joint_angles(1) = M_PI / 4; //shoulder one
    joint_angles(2) = M_PI / 2; //shoulder two
    joint_angles(3) = 0; //wrist Z
    joint_angles(4) = M_PI / 8; //wrist X
    joint_angles(5) = 0; //end effector

    // Defining joint limits
    min_joint_limits.resize(num_joints);
    max_joint_limits.resize(num_joints);
    min_joint_limits(0) = -3.14159; max_joint_limits(0) = 3.14159;
    min_joint_limits(1) = -3.14159; max_joint_limits(1) = 3.14159;
    min_joint_limits(2) = -3.14159; max_joint_limits(2) = 3.14159;
    min_joint_limits(3) = -3.14159; max_joint_limits(3) = 3.14159;
    min_joint_limits(4) = -3.14159; max_joint_limits(4) = 3.14159;
    min_joint_limits(5) = 0;        max_joint_limits(5) = 0.15;

    // Initializing the solvers
    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm_chain);

    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm_chain);
    ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_pinv>(arm_chain);
    ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR>(arm_chain, *fk_solver, *ik_solver_vel, 100, 1e-4);
    //ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(arm_chain, min_joint_limits, max_joint_limits, *fk_solver, *ik_solver_vel, 100, 1e-5);
}

KDL::Frame RobotArm::getEndEffectorPose() const {
    if (KDL::Frame end_effector_pose; fk_solver->JntToCart(joint_angles, end_effector_pose) >= 0) {
        return end_effector_pose;
    }
    // Return identity frame on error
    return KDL::Frame::Identity();
}

bool RobotArm::moveToPose(const KDL::Frame& target_pose) {
    KDL::JntArray result_angles(arm_chain.getNrOfJoints());
    const int status = ik_solver_pos->CartToJnt(joint_angles, target_pose, result_angles);

    if (status >= 0) {
        for (unsigned int i = 0; i < arm_chain.getNrOfJoints(); i++) {
            joint_angles(i) = result_angles(i);
        }


    }

    return status >= 0;
}

void RobotArm::moveToDefaultPose() {
    joint_angles(0) = 0.1;
    joint_angles(1) = 0.1;
    joint_angles(2) = 0.1;
    joint_angles(3) = 0.1;
    joint_angles(4) = 0.1;
    joint_angles(5) = 0;
}

double RobotArm::getJointAngle(const unsigned int joint_index) const {
    return joint_angles(joint_index);
}

bool RobotArm::setJointAngle(const unsigned int joint_index, const double target_angle) {
    if (joint_index >= getNumJoints()) {
        std::cerr << "Error: Invalid joint index " << joint_index << std::endl;
        return false;
    }

    if (target_angle >= min_joint_limits(joint_index) && target_angle <= max_joint_limits(joint_index)) {
        joint_angles(joint_index) = target_angle;
        return true;
    }

    return false;
}

unsigned int RobotArm::getNumJoints() const {
    return arm_chain.getNrOfJoints();
}

void RobotArm::printSegmentPositions() const {
    std::cout << "\n--- Arm Segment Positions ---" << std::endl;
    KDL::Frame segment_pose;

    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
        if (fk_solver->JntToCart(joint_angles, segment_pose, i + 1) >= 0) {
            // Get the position vector
            KDL::Vector pos = segment_pose.p;

            // Apply formatting to std::cout for this specific output
            std::cout << "Segment " << i + 1 << " Position (x, y, z): ["
                      << std::fixed << std::setprecision(5) // Set to 5 decimal places
                      << pos.x() << ", "
                      << pos.y() << ", "
                      << pos.z() << "]" << std::endl;
        } else {
            std::cerr << "Error calculating pose for segment " << i + 1 << std::endl;
        }
    }
    std::cout << "---------------------------\n" << std::endl;
}

void RobotArm::toDesmosVector() const {
    /*// 1. Generate the vector strings
    std::vector<std::string> vector_strings;
    KDL::Frame segment_pose;
    KDL::Vector start_point = KDL::Vector::Zero();

    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
        if (fk_solver->JntToCart(joint_angles, segment_pose, i + 1) >= 0) {
            KDL::Vector end_point = segment_pose.p;

            std::stringstream ss;
            ss << "var graph" << i + 1 << R"( = '\\operatorname{vector}\\left(\\left()"
               << std::fixed << std::setprecision(4)
               << start_point.x() << "," << start_point.y() << "," << start_point.z()
               << R"(\\right),\\left()"
               << end_point.x() << "," << end_point.y() << "," << end_point.z()
               << R"(\\right)\\ \\right)')";

            vector_strings.push_back(ss.str());
            start_point = end_point;
        } else {
            std::cerr << "Error calculating pose for segment " << i + 1 << std::endl;
            return; // Exit if there's an error
        }
    }

    // 2. Read the template HTML file
    std::ifstream template_file("../desmosVisualizationTemplate.html");
    if (!template_file.is_open()) {
        std::cerr << "Error: Could not open desmosVisualizationTemplate.html" << std::endl;
        return;
    }

    std::vector<std::string> lines;
    std::string line;
    while (std::getline(template_file, line)) {
        lines.push_back(line);
    }
    template_file.close();

    // 3. Replace the lines in the file content
    if (lines.size() >= 23) { // Ensure the file is long enough
        for (int i = 0; i < 5; ++i) {
            // Replace lines 19 through 23 (which are indices 18 through 22 in a 0-indexed vector)
            lines[18 + i] = vector_strings[i];
        }
    } else {
        std::cerr << "Error: HTML template is not long enough." << std::endl;
        return;
    }

    // 4. Write the new content to an output file
    std::ofstream output_file("../desmosVisualization.html");
    if (!output_file.is_open()) {
        std::cerr << "Error: Could not create desmosVisualization.html" << std::endl;
        return;
    }

    for (const auto& l : lines) {
        output_file << l << std::endl;
    }
    output_file.close();

    std::cout << "Successfully generated desmosVisualization.html" << std::endl;*/


    std::cout << "\n--- Desmos Vector Output ---" << std::endl;
    KDL::Frame segment_pose;
    // The starting point for the first vector is the origin
    KDL::Vector start_point = KDL::Vector::Zero();

    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
        if (fk_solver->JntToCart(joint_angles, segment_pose, i + 1) >= 0) {
            KDL::Vector end_point = segment_pose.p;

            // Set output to fixed, 4 decimal places for clean output
            std::cout << std::fixed << std::setprecision(5)
                      << R"(\operatorname{vector}\left(\left()"
                      << start_point.x() << "," << start_point.y() << "," << start_point.z()
                      << "\\right),\\left("
                      << end_point.x() << "," << end_point.y() << "," << end_point.z()
                      << R"(\right)\ \right))" << std::endl;

            // The endpoint of this segment is the start point of the next one
            start_point = end_point;
        } else {
            std::cerr << "Error calculating pose for segment " << i + 1 << std::endl;
        }
    }
    std::cout << "--------------------------\n" << std::endl;
}
