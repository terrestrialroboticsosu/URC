/**
 * @file robotArm.h
 * @brief Manages the kinematics of a 5-axis robotic arm using Orocos KDL.
 */

#pragma once

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <memory> // For std::unique_ptr

/**
 * @class RobotArm
 * @brief Encapsulates the kinematic model and solvers for a 5-axis arm.
 *
 * This class uses Orocos KDL to define the arm's structure and provides methods
 * for forward kinematics (calculating end-effector pose from joint angles) and
 * inverse kinematics (calculating joint angles for a desired end-effector pose).
 */
class RobotArm {
public:
    /**
     * @brief Constructs a new RobotArm object and initializes its kinematic chain.
     */
    RobotArm();

    /**
     * @brief Calculates the end-effector's position and orientation.
     * @param joint_angles A KDL JntArray containing the current angle of each joint in radians.
     * @return A KDL Frame representing the pose (position and orientation) of the end-effector.
     */
    KDL::Frame getEndEffectorPose(const KDL::JntArray& joint_angles);

    /**
     * @brief Calculates the joint angles required to reach a target pose.
     * @param target_pose The desired pose (position and orientation) for the end-effector.
     * @param current_angles The current joint angles, used as an initial guess for the solver.
     * @param result_angles A KDL JntArray that will be filled with the calculated joint angles.
     * @return True if a solution was found, false otherwise.
     */
    bool moveToPose(const KDL::Frame& target_pose, const KDL::JntArray& current_angles, KDL::JntArray& result_angles);

    /**
     * @brief Sets a specific joint to a target angle.
     * @param joint_angles The JntArray representing the arm's state. This will be modified directly.
     * @param joint_index The index of the joint to modify (0-based).
     * @param target_angle The desired angle for the joint in radians.
     * @return True if the target angle is within limits, false otherwise.
     */
    bool setJointAngle(KDL::JntArray& joint_angles, unsigned int joint_index, double target_angle);

    /**
     * @brief Gets the number of joints in the arm's kinematic chain.
     * @return The number of joints.
     */
    unsigned int getNumJoints() const;

    /**
     * @brief Prints the Cartesian (x, y, z) position of each segment's tip.
     * @param joint_angles The current joint angles for which to calculate the positions.
     */
    void printSegmentPositions(const KDL::JntArray& joint_angles);

    /**
     * @brief Prints the arm's segments as vectors in a Desmos-compatible format.
     *
     * This method is only intended to be used with the webpage 'https://www.desmos.com/3d' for visualization
     * A forward kinematic visualization can be found at this webpage 'https://www.desmos.com/3d/6kwzyiauy0'
     * @param joint_angles The current joint angles for which to generate the vectors.
     *
     */
    void toDesmosVector(const KDL::JntArray& joint_angles);

private:
    KDL::Chain arm_chain; /**< The kinematic chain representing the robotic arm. */

    // Joint Limits
    KDL::JntArray min_joint_limits;
    KDL::JntArray max_joint_limits;

    // Using smart pointers to manage the lifetime of the solvers
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver; /**< Forward kinematics solver. */
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel;  /**< Inverse kinematics velocity solver. */
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos;    /**< Inverse kinematics position solver (limits). */
};