#include "research_interface.h"
#include "shared_memory.h"
#include "shared_memory_manager.h"
#include "gtest/gtest.h"

TEST(ResearchInterfaceTest, joints) {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> m(harmony::dataFromHarmonyMemId, true);
    ASSERT_TRUE(m.init());

    harmony::ResearchInterface e;
    ASSERT_TRUE(e.init());

    for (size_t i = 0; i < harmony::armJointCount; i++) {
        m.data->leftArmJointStates[i] = {(double)i, (double)i + 5};
        m.data->rightArmJointStates[i] = {(double)i * 100, (double)i * 200};
    }

    for (size_t i = 0; i < harmony::armJointCount; i++) {
        ASSERT_EQ(e.joints().leftArm.getOrderedStates()[i].position_rad, (double)i);
        ASSERT_EQ(e.joints().leftArm.getOrderedStates()[i].torque_Nm, (double)i + 5);

        ASSERT_EQ(e.joints().rightArm.getOrderedStates()[i].position_rad, (double)i * 100);
        ASSERT_EQ(e.joints().rightArm.getOrderedStates()[i].torque_Nm, (double)i * 200);
    }
}

TEST(ResearchInterfaceTest, sizes) {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> m(harmony::dataFromHarmonyMemId, true);
    ASSERT_TRUE(m.init());

    harmony::ResearchInterface e;
    ASSERT_TRUE(e.init());

    std::array<double, harmony::armSizeCount> left = {1, 2, 3, 4};
    std::array<double, harmony::armSizeCount> right = {5, 6, 7, 8};

    for (size_t i = 0; i < harmony::armSizeCount; i++) {
        m.data->leftArmSizes_mm[i] = left[i];
        m.data->rightArmSizes_mm[i] = right[i];
    }

    for (size_t i = 0; i < harmony::armSizeCount; i++) {
        ASSERT_EQ(e.sizes().leftArm.getOrderedSizes_mm()[i], left[i]);
        ASSERT_EQ(e.sizes().rightArm.getOrderedSizes_mm()[i], right[i]);
    }
}

void assertPoseEquality(harmony::Pose a, harmony::Pose b) {
    ASSERT_EQ(a.position_mm.x, b.position_mm.x);
    ASSERT_EQ(a.position_mm.y, b.position_mm.y);
    ASSERT_EQ(a.position_mm.z, b.position_mm.z);
    ASSERT_EQ(a.orientation.x, b.orientation.x);
    ASSERT_EQ(a.orientation.y, b.orientation.y);
    ASSERT_EQ(a.orientation.z, b.orientation.z);
    ASSERT_EQ(a.orientation.w, b.orientation.w);
}

TEST(ResearchInterfaceTest, poses) {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> m(harmony::dataFromHarmonyMemId, true);
    ASSERT_TRUE(m.init());

    harmony::ResearchInterface e;
    ASSERT_TRUE(e.init());

    harmony::Pose leftPose{{1, 2, 3}, {4, 5, 6, 7}};
    harmony::Pose rightPose{{11, 12, 13}, {14, 15, 16, 17}};

    m.data->leftEndEffectorPose = leftPose;
    m.data->rightEndEffectorPose = rightPose;

    assertPoseEquality(e.poses().leftEndEffector, leftPose);
    assertPoseEquality(e.poses().rightEndEffector, rightPose);
}

TEST(ResearchInterfaceTest, readDigitalInput) {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> m(harmony::dataFromHarmonyMemId, true);
    ASSERT_TRUE(m.init());

    harmony::ResearchInterface e;
    ASSERT_TRUE(e.init());

    ASSERT_FALSE(e.readDigitalInput()[0]);
    ASSERT_FALSE(e.readDigitalInput()[1]);
    m.data->digitalInput[0] = true;
    m.data->digitalInput[1] = true;
    ASSERT_TRUE(e.readDigitalInput()[0]);
    ASSERT_TRUE(e.readDigitalInput()[1]);
}

TEST(ResearchInterfaceTest, writeDigitalOutput) {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> m(harmony::dataFromHarmonyMemId, true);
    harmony::SharedMemoryManager<harmony::DataToHarmony> d(harmony::dataToHarmonyMemId, true);

    ASSERT_TRUE(m.init());
    ASSERT_TRUE(d.init());

    harmony::ResearchInterface e;
    ASSERT_TRUE(e.init());

    ASSERT_EQ(d.data->digitalOutput[0], 0);
    ASSERT_EQ(d.data->digitalOutput[1], 0);

    e.writeDigitalOutput0(1);
    ASSERT_EQ(d.data->digitalOutput[0], 1);
    ASSERT_EQ(d.data->digitalOutput[1], 0);

    e.writeDigitalOutput1(1);
    ASSERT_EQ(d.data->digitalOutput[0], 1);
    ASSERT_EQ(d.data->digitalOutput[1], 1);

    e.writeDigitalOutput({0, 0});
    ASSERT_EQ(d.data->digitalOutput[0], 0);
    ASSERT_EQ(d.data->digitalOutput[1], 0);

    e.writeDigitalOutput({1, 0});
    ASSERT_EQ(d.data->digitalOutput[0], 1);
    ASSERT_EQ(d.data->digitalOutput[1], 0);
}
