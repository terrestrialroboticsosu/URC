/**
 * @file robotArm.h
 * @brief Manages the kinematics of a 5-axis robotic arm using Orocos KDL.
 */

#pragma once

#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
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
     * @return A KDL Frame representing the pose (position and orientation) of the end-effector.
     */
    [[nodiscard]] KDL::Frame getEndEffectorPose() const;

    /**
     * @brief Calculates the joint angles required to reach a target pose, and updates the joint angles array.
     * @param target_pose The desired pose (position and orientation) for the end-effector.
     * @return True if a solution was found, false otherwise.
     */
    bool moveToPose(const KDL::Frame &target_pose);

    /**
    * @brief Moves the robot arm to the default position.
    */
    void moveToDefaultPose();

    /**
     * @brief Gets a specific joint's angle.
     * @param joint_index The index of the joint to modify (0-based).
     * @return Angle of the specified joint.
     */
    [[nodiscard]] double getJointAngle(unsigned int joint_index) const;

    /**
     * @brief Sets a specific joint to a target angle.
     * @param joint_index The index of the joint to modify (0-based).
     * @param target_angle The desired angle for the joint in radians.
     * @return True if the target angle is within limits, false otherwise.
     */
    bool setJointAngle(unsigned int joint_index, double target_angle);

    /**
     * @brief Gets the number of joints in the arm's kinematic chain.
     * @return The number of joints.
     */
    [[nodiscard]] unsigned int getNumJoints() const;

    /**
     * @brief Prints the Cartesian (x, y, z) position of each segment's tip.
     */
    void printSegmentPositions() const;

    /**
     * @brief Prints the arm's segments as vectors in a Desmos-compatible format.
     *
     * This method is only intended to be used with the webpage 'https://www.desmos.com/3d' for visualization
     * A forward kinematic visualization can be found at this webpage 'https://www.desmos.com/3d/6kwzyiauy0'
     */
    void toDesmosVector() const;

private:
    KDL::Chain arm_chain; /**< The kinematic chain representing the robotic arm. */

    KDL::JntArray joint_angles; /**< The array of angles for each join of the robot arm. */

    // Joint Limits
    KDL::JntArray min_joint_limits;
    KDL::JntArray max_joint_limits;

    // Using smart pointers to manage the lifetime of the solvers
    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver; /**< Forward kinematics solver. */
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_solver_vel;  /**< Inverse kinematics velocity solver. */
    std::unique_ptr<KDL::ChainIkSolverPos_NR> ik_solver_pos;    /**< Inverse kinematics position solver (limits). */
    //std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_solver_pos;    /**< Inverse kinematics position solver (limits). */
};