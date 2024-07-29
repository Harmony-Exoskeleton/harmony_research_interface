#include "sizes.h"
#include "gtest/gtest.h"
#include <array>

TEST(ArmSizesTest, getOrderedSizes) {
    std::array<double, harmony::armSizeCount> s = {1, 2, 3, 4};
    auto sizes = harmony::ArmSizes(s);
    auto ordered = sizes.getOrderedSizes_mm();

    for (size_t i = 0; i < s.size(); i++) {
        ASSERT_EQ(s[i], ordered[i]);
    }
}

TEST(ArmSizesTest, getSize_mm) {
    std::array<double, harmony::armSizeCount> s = {1, 2, 3, 4};
    auto sizes = harmony::ArmSizes(s);

    ASSERT_EQ(s[0], sizes.getSize_mm(harmony::ArmSize::shoulderHeight));
    ASSERT_EQ(s[1], sizes.getSize_mm(harmony::ArmSize::shoulderBreadth));
    ASSERT_EQ(s[2], sizes.getSize_mm(harmony::ArmSize::humeralLength));
    ASSERT_EQ(s[3], sizes.getSize_mm(harmony::ArmSize::forearmLength));
}
