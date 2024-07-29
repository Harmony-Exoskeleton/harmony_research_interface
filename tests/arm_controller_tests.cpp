#include "research_interface.h"
#include "shared_memory.h"
#include "shared_memory_manager.h"
#include "gtest/gtest.h"

TEST(ControllerTest, initialMode) {
    harmony::SharedMemoryManager<harmony::ArmControllerState> m(harmony::leftArmControllerMemId, true);
    ASSERT_TRUE(m.init());
    auto c = harmony::ResearchInterface::makeLeftArmController();
    ASSERT_TRUE(c->init());
    ASSERT_EQ(c->getMode(), harmony::ArmController::Mode::harmony);
    ASSERT_EQ(c->getArmJointsOverride(), nullptr);
}

TEST(ControllerTest, jointsOverride) {
    harmony::SharedMemoryManager<harmony::ArmControllerState> m(harmony::rightArmControllerMemId, true);
    ASSERT_TRUE(m.init());

    auto c = harmony::ResearchInterface::makeRightArmController();
    ASSERT_TRUE(c->init());

    ASSERT_EQ(c->getMode(), harmony::ArmController::Mode::harmony);
    ASSERT_EQ(m.data->mode, harmony::ArmController::Mode::harmony);

    std::array<harmony::JointOverride, harmony::armJointCount> overrides;
    for (size_t i = 0; i < harmony::armJointCount; i++) {
        overrides[i] = {double(i), double(i * 100)};
    }

    auto override = harmony::ArmJointsOverride(overrides);
    c->setJointsOverride(override);

    ASSERT_EQ(c->getMode(), harmony::ArmController::Mode::jointsOverride);
    ASSERT_EQ(m.data->mode, harmony::ArmController::Mode::jointsOverride);

    ASSERT_TRUE(c->getArmJointsOverride()->areShoulderConstraintsEnabled());
    ASSERT_TRUE(m.data->shoulderConstraintsEnabled);

    auto retrievedOverrides = c->getArmJointsOverride();
    ASSERT_NE(retrievedOverrides, nullptr);

    for (size_t i = 0; i < harmony::armJointCount; i++) {
        ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredPosition_rad, double(i));
        ASSERT_EQ(m.data->jointsOverride[i].desiredPosition_rad, double(i));
        ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredStiffness_Nm_per_rad, double(i * 100));
        ASSERT_EQ(m.data->jointsOverride[i].desiredStiffness_Nm_per_rad, double(i * 100));
    }

    retrievedOverrides->applyOverrides({{harmony::ArmJoint::elbowFlexion, {5555, 4444}}});
    retrievedOverrides->disableShoulderConstraints();

    c->setJointsOverride(*retrievedOverrides);
    ASSERT_FALSE(c->getArmJointsOverride()->areShoulderConstraintsEnabled());

    for (size_t i = 0; i < harmony::armJointCount; i++) {
        if (i == static_cast<int>(harmony::ArmJoint::elbowFlexion)) {
            ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredPosition_rad, 5555);
            ASSERT_EQ(m.data->jointsOverride[i].desiredPosition_rad, 5555);
            ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredStiffness_Nm_per_rad, 4444);
            ASSERT_EQ(m.data->jointsOverride[i].desiredStiffness_Nm_per_rad, 4444);
        } else {
            ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredPosition_rad, double(i));
            ASSERT_EQ(m.data->jointsOverride[i].desiredPosition_rad, double(i));
            ASSERT_EQ(retrievedOverrides->getOrderedOverrides()[i].desiredStiffness_Nm_per_rad, double(i * 100));
            ASSERT_EQ(m.data->jointsOverride[i].desiredStiffness_Nm_per_rad, double(i * 100));
        }
    }
}