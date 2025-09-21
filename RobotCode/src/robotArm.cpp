/**
 * @file robotArm.cpp
 * @brief Implementation of the RobotArm class.
 */

#include "robotArm.h"
#include <iostream>
#include <kdl/frames_io.hpp> // For printing frames and vectors
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

    // --- Placeholder Example for a 5-Axis Arm ---

    // Base to Joint 1 (e.g., rotation around the base)
    // Frame(Vector(dx, dy, dz)): dx, dy, dz are the distances in meters.
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotZ),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.2)))); // 20cm up from base

    // Joint 1 to Joint 2 (e.g., shoulder pitch)
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.5)))); // 50cm arm segment

    // Joint 2 to Joint 3 (e.g., elbow pitch)
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.4)))); // 40cm forearm segment

    // Joint 3 to Joint 4 (e.g., wrist roll)
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotX),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.1)))); // 10cm to wrist pitch

    // Joint 4 to Joint 5 (wrist pitch/end-effector)
    arm_chain.addSegment(KDL::Segment(KDL::Joint(KDL::Joint::RotY),
                                      KDL::Frame(KDL::Vector(0.0, 0.0, 0.05)))); // 5cm to end-effector tip

    // --- DEFINE JOINT LIMITS (in Radians) ---
    // You must get these values from your arm's physical design.
    // Pi is approx 3.14159 radians (180 degrees).
    unsigned int num_joints = arm_chain.getNrOfJoints();
    min_joint_limits.resize(num_joints);
    max_joint_limits.resize(num_joints);

    // Example limits (e.g., +/- 180 degrees) - REPLACE WITH YOUR REAL LIMITS
    min_joint_limits(0) = -3.14159; max_joint_limits(0) = 3.14159; // Joint 0
    min_joint_limits(1) = -3.14159; max_joint_limits(1) = 3.14159; // Joint 1
    min_joint_limits(2) = -3.14159; max_joint_limits(2) = 3.14159; // Joint 2
    min_joint_limits(3) = -3.14159; max_joint_limits(3) = 3.14159; // Joint 3
    min_joint_limits(4) = -3.14159; max_joint_limits(4) = 3.14159; // Joint 4

    // Initialize the solvers
    fk_solver = std::make_unique<KDL::ChainFkSolverPos_recursive>(arm_chain);
    ik_solver_vel = std::make_unique<KDL::ChainIkSolverVel_pinv>(arm_chain);
    ik_solver_pos = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(arm_chain, min_joint_limits, max_joint_limits, *fk_solver, *ik_solver_vel, 100, 1e-5);
}

KDL::Frame RobotArm::getEndEffectorPose(const KDL::JntArray& joint_angles) {
    KDL::Frame end_effector_pose;
    if (fk_solver->JntToCart(joint_angles, end_effector_pose) >= 0) {
        return end_effector_pose;
    }
    // Return identity frame on error
    return KDL::Frame::Identity();
}

bool RobotArm::moveToPose(const KDL::Frame& target_pose, const KDL::JntArray& current_angles, KDL::JntArray& result_angles) {
    int status = ik_solver_pos->CartToJnt(current_angles, target_pose, result_angles);
    return status >= 0;
}

bool RobotArm::setJointAngle(KDL::JntArray& joint_angles, unsigned int joint_index, double target_angle) {
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

void RobotArm::printSegmentPositions(const KDL::JntArray& joint_angles) {
    std::cout << "\n--- Arm Segment Positions ---" << std::endl;
    KDL::Frame segment_pose;

    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
        if (fk_solver->JntToCart(joint_angles, segment_pose, i + 1) >= 0) {
            // Get the position vector
            KDL::Vector pos = segment_pose.p;

            // Apply formatting to std::cout for this specific output
            std::cout << "Segment " << i + 1 << " Position (x, y, z): ["
                      << std::fixed << std::setprecision(4) // Set to 4 decimal places
                      << pos.x() << ", "
                      << pos.y() << ", "
                      << pos.z() << "]" << std::endl;
        } else {
            std::cerr << "Error calculating pose for segment " << i + 1 << std::endl;
        }
    }
    std::cout << "---------------------------\n" << std::endl;
}

void RobotArm::toDesmosVector(const KDL::JntArray& joint_angles) {
    std::cout << "\n--- Desmos Vector Output ---" << std::endl;
    KDL::Frame segment_pose;
    // The starting point for the first vector is the origin
    KDL::Vector start_point = KDL::Vector::Zero();

    for (unsigned int i = 0; i < arm_chain.getNrOfSegments(); i++) {
        if (fk_solver->JntToCart(joint_angles, segment_pose, i + 1) >= 0) {
            KDL::Vector end_point = segment_pose.p;

            // Set output to fixed, 4 decimal places for clean output
            std::cout << std::fixed << std::setprecision(4)
                      << "\\operatorname{vector}\\left(\\left("
                      << start_point.x() << "," << start_point.y() << "," << start_point.z()
                      << "\\right),\\left("
                      << end_point.x() << "," << end_point.y() << "," << end_point.z()
                      << "\\right)\\ \\right)" << std::endl;

            // The endpoint of this segment is the start point of the next one
            start_point = end_point;
        } else {
            std::cerr << "Error calculating pose for segment " << i + 1 << std::endl;
        }
    }
    std::cout << "--------------------------\n" << std::endl;
}