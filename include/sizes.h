#ifndef HARMONY_SIZES_H
#define HARMONY_SIZES_H

#include <array>

namespace harmony {

enum class ArmSize {
    shoulderHeight = 0,
    shoulderBreadth = 1,
    humeralLength = 2,
    forearmLength = 3,
};

constexpr int armSizeCount = static_cast<int>(ArmSize::forearmLength) + 1;

class ArmSizes {
private:
    std::array<double, armSizeCount> armSizes_mm;

public:
    explicit ArmSizes(std::array<double, armSizeCount> armSizes_mm)
        : armSizes_mm(armSizes_mm) {}
    // provides all sizes in order from shoulderHeight to forearmLength (see ArmSize enum)
    std::array<double, armSizeCount> getOrderedSizes_mm() const { return armSizes_mm; }
    double getSize_mm(ArmSize size) const { return armSizes_mm[static_cast<int>(size)]; }
};

struct Sizes {
    ArmSizes leftArm;
    ArmSizes rightArm;
};

} // namespace harmony

#endif