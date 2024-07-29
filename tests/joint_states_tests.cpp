#include "joint_states.h"
#include "gtest/gtest.h"
#include <array>

TEST(ArmJointStatesTest, getOrderedSizes) {
    std::array<harmony::JointState, harmony::armJointCount> states;

    for (size_t i = 0; i < states.size(); i++) {
        states[i] = {(double)i, -((double)i)};
    }

    auto jointStates = harmony::ArmJointStates(states);
    auto ordered = jointStates.getOrderedStates();

    for (size_t i = 0; i < states.size(); i++) {
        ASSERT_EQ(states[i].position_rad, ordered[i].position_rad);
        ASSERT_EQ(states[i].torque_Nm, ordered[i].torque_Nm);
    }
}

TEST(ArmJointStatesTest, getSize_mm) {
    std::array<harmony::JointState, harmony::armJointCount> states;

    for (size_t i = 0; i < states.size(); i++) {
        states[i] = {(double)i, -((double)i)};
    }

    auto jointStates = harmony::ArmJointStates(states);

    ASSERT_EQ(states[0].position_rad, jointStates.getState(harmony::ArmJoint::scapularElevation).position_rad);
    ASSERT_EQ(states[0].torque_Nm, jointStates.getState(harmony::ArmJoint::scapularElevation).torque_Nm);

    ASSERT_EQ(states[1].position_rad, jointStates.getState(harmony::ArmJoint::scapularProtraction).position_rad);
    ASSERT_EQ(states[1].torque_Nm, jointStates.getState(harmony::ArmJoint::scapularProtraction).torque_Nm);

    ASSERT_EQ(states[2].position_rad, jointStates.getState(harmony::ArmJoint::shoulderAbduction).position_rad);
    ASSERT_EQ(states[2].torque_Nm, jointStates.getState(harmony::ArmJoint::shoulderAbduction).torque_Nm);

    ASSERT_EQ(states[3].position_rad, jointStates.getState(harmony::ArmJoint::shoulderRotation).position_rad);
    ASSERT_EQ(states[3].torque_Nm, jointStates.getState(harmony::ArmJoint::shoulderRotation).torque_Nm);

    ASSERT_EQ(states[4].position_rad, jointStates.getState(harmony::ArmJoint::shoulderFlexion).position_rad);
    ASSERT_EQ(states[4].torque_Nm, jointStates.getState(harmony::ArmJoint::shoulderFlexion).torque_Nm);

    ASSERT_EQ(states[5].position_rad, jointStates.getState(harmony::ArmJoint::elbowFlexion).position_rad);
    ASSERT_EQ(states[5].torque_Nm, jointStates.getState(harmony::ArmJoint::elbowFlexion).torque_Nm);

    ASSERT_EQ(states[6].position_rad, jointStates.getState(harmony::ArmJoint::wristPronation).position_rad);
    ASSERT_EQ(states[6].torque_Nm, jointStates.getState(harmony::ArmJoint::wristPronation).torque_Nm);
}
