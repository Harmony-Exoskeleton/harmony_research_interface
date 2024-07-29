#ifndef HARMONY_JOINT_STATES_H
#define HARMONY_JOINT_STATES_H

#include <array>

namespace harmony {

enum class ArmJoint {
    scapularElevation = 0,
    scapularProtraction = 1,
    shoulderAbduction = 2,
    shoulderRotation = 3, // internal rotation
    shoulderFlexion = 4,
    elbowFlexion = 5,
    wristPronation = 6,
};

constexpr int armJointCount = static_cast<int>(ArmJoint::wristPronation) + 1;

struct JointState {
    double position_rad;
    double torque_Nm;
};

class ArmJointStates {
private:
    std::array<JointState, armJointCount> states;

public:
    explicit ArmJointStates(std::array<JointState, armJointCount> states)
        : states(states) {}
    // provides all JointStates in order from scapularElevation to writePronation
    std::array<JointState, armJointCount> getOrderedStates() const { return states; }
    JointState getState(ArmJoint joint) const { return states[static_cast<int>(joint)]; }
};

struct JointStates {
    ArmJointStates leftArm;
    ArmJointStates rightArm;
};

} // namespace harmony

#endif