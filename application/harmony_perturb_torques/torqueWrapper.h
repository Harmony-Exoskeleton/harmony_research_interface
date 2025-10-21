#ifndef torqueWrapper_H
#define torqueWrapper_H


// includes
#include "research_interface.h"
#include <Eigen/Dense>
#include <array> 
#include <iostream>
#include <chrono> 

#define s400Stiffness_Nm_p_rad 50.0 //15.0 // desired joint stiffness for series 400 [shoulder] motors(max is 50)
#define s600Stiffness_Nm_p_rad 30.0 //15.0 // desired joint stiffness for series 600 [elbow] motors (max is 30)
#define s700Stiffness_Nm_p_rad 3.0 // desired joint stiffness for series 700 [wrist] motors(max is 3)

#define nCols 2 * harmony::armJointCount + 1 // number of columns in dataset
#define N_JOINTS 7



/******************************************************************************************
 * STRUCTS
 *****************************************************************************************/
struct AllArmsOverrides {
    harmony::ArmJointsOverride leftOverrides;
    harmony::ArmJointsOverride rightOverrides;
#ifdef TORSO_MODS
    harmony::TorsoJointsOverride torsoOverrides;
#endif
};

/******************************************************************************************
 * CLASS
 *****************************************************************************************/
class harmony_robot {
public:
    double jointStiffness(int joint_idx);

    AllArmsOverrides data2override(std::array<double, nCols> data);

    AllArmsOverrides convert_torque_to_override(Eigen::VectorXd desired_torque,
        harmony::ResearchInterface* info,
        char side);

    void set_ordered_states(harmony::ResearchInterface* info);
    Eigen::VectorXd get_position_rads(char side) {
        if (side == 'L') {
            return left_position_radians;
        } else if (side == 'R') {
            return right_position_radians;
        } else if (side == 'B') {
            return both_position_radians;
        } else {
            // cry
            return both_position_radians;
        }
    };

    Eigen::VectorXd get_torque_Nm(char side) {
        if (side == 'L') {
            return left_torque_Nm;
        } else if (side == 'R') {
            return right_torque_Nm;
        } else if (side == 'B') {
            return both_torque_Nm;
        } else {
            // cry
            return both_torque_Nm;
        }
    };

    std::array<double,nCols> setTorqueVector();

private:
    float measured_position_radians;
    float measured_torque_Nm;

    Eigen::VectorXd left_position_radians = Eigen::VectorXd::Zero(N_JOINTS);
    Eigen::VectorXd right_position_radians = Eigen::VectorXd::Zero(N_JOINTS);
    Eigen::VectorXd both_position_radians = Eigen::VectorXd::Zero(N_JOINTS * 2);

    Eigen::VectorXd left_torque_Nm = Eigen::VectorXd::Zero(N_JOINTS);
    Eigen::VectorXd right_torque_Nm = Eigen::VectorXd::Zero(N_JOINTS);
    Eigen::VectorXd both_torque_Nm = Eigen::VectorXd::Zero(N_JOINTS * 2);
};

/******************************************************************************************
 * FUNCTION
 *****************************************************************************************/
/**
 * @brief returns the desired joint stiffness based on the actuator type
 * Joints <5 are all series 400, joint 5 is the elbow series 500, and
 * joint 6 is the wrist series 700 joint.
 * @param joint_idx current index for joint
 * @return double the desired joint stiffness
 */
double jointStiffness(int joint_idx) {
    double stiffness;
    harmony::ArmJoint joint = harmony::ArmJoint(joint_idx);

    switch (joint) {
        case harmony::ArmJoint::elbowFlexion:
            stiffness = s600Stiffness_Nm_p_rad;
            break;
        case harmony::ArmJoint::wristPronation:
            stiffness = s700Stiffness_Nm_p_rad;
            break;
#ifdef WRIST_MODS
        case harmony::ArmJoint::wristAbduction:
            stiffness = WD_SCALING * s700Stiffness_Nm_p_rad;
            break;
        case harmony::ArmJoint::wristFlexion:
            stiffness = WF_SCALING * s700Stiffness_Nm_p_rad;
            break;
#endif
        default:
            stiffness = s400Stiffness_Nm_p_rad;
            break;
    }

    return stiffness;
}

/**
 * @brief Takes a data array and converts it to override format
 *
 * @param data read data line from log file, parsed to array
 * @return AllArmsOverrides studt containing left and right overrides
 */
AllArmsOverrides data2override(std::array<double, nCols> data) {
    std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;
    for (int i = 0; i < harmony::armJointCount; i++) {
        leftOverrides[i] = {data[i + 1], jointStiffness(i)};
        rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i)};
    }

#ifdef TORSO_MODS
    std::array<harmony::JointOverride, harmony::torsoJointCount> torsoOverrides;
    for (int i = 0; i < harmony::torsoJointCount; i++) {
        torsoOverrides[i] = {data[i + harmony::armJointCount * 2 + 1], jointStiffness(i)};
    }
#endif

    return {harmony::ArmJointsOverride(leftOverrides),
        harmony::ArmJointsOverride(rightOverrides)
#ifdef TORSO_MODS
            ,
        harmony::TorsoJointsOverride(torsoOverrides)
#endif
    };
}

/**
 * @brief Wrapper function that converts torques to impedance control setpoints for research interface
 *
 * @param desired_torque vector of desired torque values
 * @param info ResearchInterface object that contains current joint positions
 *
 * @return AllArmsOverrides object with impedance control setpoints for both arms
 *
 */
// std::array<double, nCols> convert_torque_to_override(std::array<double, nCols> desired_torque,
AllArmsOverrides convert_torque_to_override(std::array<double, nCols> desired_torque,
    harmony::ResearchInterface* info,
    char side) {

    std::array<harmony::JointState, harmony::armJointCount> states_left = info->joints().leftArm.getOrderedStates();
    std::array<harmony::JointState, harmony::armJointCount> states_right = info->joints().rightArm.getOrderedStates();

    // std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    // std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;

    std::array<double, nCols> data;
    data[0] = 0.0;

    int mNm_2_Nm = 1000;

    // sanity's
    // data = {0, L(1x7), R(1x7)}
    switch (side) {
        case 'L': // LEFT SIDE
            // std::cout << "left side\n";
            for (int i = 0; i < harmony::armJointCount; i++) {
                // override left
                data[i + 1] = desired_torque[i] / (mNm_2_Nm * jointStiffness(i)) + states_left[i].position_rad;
                // keep right
                data[i + harmony::armJointCount + 1] = states_right[i].position_rad;
            }
            break;

        // This isn't it
        case 'R': // RIGHT SIDE
            // std::cout << "right side\n";
            for (int i = 0; i < harmony::armJointCount; i++) {
                // keep left
                data[i + 1] = states_left[i].position_rad;
                // override right
                data[i + harmony::armJointCount + 1] =
                    desired_torque[i] / (mNm_2_Nm * jointStiffness(i)) + states_right[i].position_rad;
            }
            break;

        case 'B': // both
            // std::cout << "Both \n";
            for (int i = 0; i < harmony::armJointCount; i++) {
                // left
                data[i + 1] = desired_torque[i] / (mNm_2_Nm * jointStiffness(i)) + states_left[i].position_rad;
                // right
                data[i + harmony::armJointCount + 1] =
                    desired_torque[i + harmony::armJointCount] / (mNm_2_Nm * jointStiffness(i)) +
                    states_right[i].position_rad;
            }
            break;

        default:
            std::cout << "nope \n";
            break;
    }
    auto override = data2override(data);
    return override;

    // // TODO: call data2overrides instead
    // // for (int i = 0; i < harmony::armJointCount; i++) {
    // //     rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i)};
    // //     leftOverrides[i] = {data[i + 1], jointStiffness(i)};
    // // }
    // auto overrides = data2override(data); // test how this works ?

    // // Print Data for testing
    // std::cout << "test" << std::endl;
    // for (int i = 0; i < sizeof(data) / sizeof(data[0]); i++) {
    //     std::cout << data[i] << ", ";
    // }
    // std::cout << std::endl;

    // return {harmony::ArmJointsOverride(overrides.leftOverrides),
    // harmony::ArmJointsOverride(overrides.rightOverrides)};
}


// *** fix and comment *** //
void harmony_robot::set_ordered_states(harmony::ResearchInterface* info) {
    auto leftStates = info->joints().leftArm.getOrderedStates();
    auto rightStates = info->joints().rightArm.getOrderedStates();

    for (int i = 0; i < N_JOINTS; i++) {
        measured_position_radians = leftStates[i].position_rad;
        left_position_radians[i] = measured_position_radians;

        measured_torque_Nm = leftStates[i].torque_Nm;
        left_torque_Nm[i] = measured_torque_Nm;
        
        measured_position_radians = rightStates[i].position_rad;
        right_position_radians[i] = measured_position_radians;

        measured_torque_Nm = rightStates[i].torque_Nm;
        right_torque_Nm[i] = measured_torque_Nm;
    }
}


std::array<double,nCols> harmony_robot::setTorqueVector() {
    std::array<double,nCols> torqueVector;

    while (true) {
        std::cout << "Set Torque (mNm) Vector (Enter all " << harmony::armJointCount*2 << " values separated by spaces):\n";

        std::string userInput;
        if (std::getline(std::cin, userInput)) {
            std::istringstream iss(userInput);
            int jointIndex = 0;

            while (jointIndex < harmony::armJointCount && iss >> torqueVector[jointIndex]) {
                ++jointIndex;
            }

            if (jointIndex == harmony::armJointCount) {
                // Valid input, break out of the loop
                break;
            } else {
                std::cout << "Invalid input! Please enter exactly " << harmony::armJointCount << " numeric values.\n";
            }
        } else {
            // Handle input error (e.g., end of file)
            std::cerr << "Error reading input. Exiting...\n";
            std::exit(EXIT_FAILURE);
        }
    }

    return torqueVector;
}

#endif // torqueWrapper_H