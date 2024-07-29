/** @file
 * @brief Declares Harmony SHR's external research interface.
 *
 */

#ifndef HARMONY_SHARED_MEMORY_H
#define HARMONY_SHARED_MEMORY_H

#include "arm_controller.h"
#include "joint_states.h"
#include "sizes.h"

namespace harmony {

constexpr int dataFromHarmonyMemId = 222;
constexpr int dataToHarmonyMemId = 223;

constexpr int leftArmControllerMemId = 233;
constexpr int rightArmControllerMemId = 234;

struct DataFromHarmony {
    JointState leftArmJointStates[armJointCount];
    JointState rightArmJointStates[armJointCount];
    double leftArmSizes_mm[armSizeCount];
    double rightArmSizes_mm[armSizeCount];
    Pose leftEndEffectorPose;
    Pose rightEndEffectorPose;
    bool digitalInput[2];
};

struct DataToHarmony {
    bool digitalOutput[2];
};

struct ArmControllerState {
    ArmController::Mode mode = ArmController::Mode::harmony;
    JointOverride jointsOverride[armJointCount];
    bool shoulderConstraintsEnabled = true; // only relevant when mode == jointsOverride
    bool gravityTorqueDisabled = false;
    bool SHRTorqueDisabled = false;
};

} // namespace harmony

#endif