#ifndef HARMONY_OVERRIDES_H
#define HARMONY_OVERRIDES_H

#include "joint_states.h"
#include "pose.h"
#include <array>
#include <map>

namespace harmony {

struct JointOverride {
    double desiredPosition_rad;
    double desiredStiffness_Nm_per_rad;
    double desiredTorque_Nm;
};

class ArmJointsOverride {
private:
    std::array<JointOverride, armJointCount> overrides;
    bool shoulderConstraintsEnabled;
    bool gravityTorqueDisabled = false;
    bool SHRTorqueDisabled = false;

public:
    explicit ArmJointsOverride(std::array<JointOverride, armJointCount> overrides,
        bool shoulderConstraintsEnabled = true,
        bool gravityTorqueDisabled = false,
        bool SHRTorqueDisabled = false)
        : overrides(overrides)
        , shoulderConstraintsEnabled(shoulderConstraintsEnabled)
        , gravityTorqueDisabled(gravityTorqueDisabled)
        , SHRTorqueDisabled(SHRTorqueDisabled) {}

    // Get a copy of the current overrides.
    std::array<JointOverride, armJointCount> getOrderedOverrides() const { return overrides; }

    // Add/overwrite specified override. All other joint overrides will remain.
    void setOverride(ArmJoint joint, JointOverride override) { overrides[static_cast<int>(joint)] = override; }

    // Replace all current overrides with the specified overrides.
    void setOrderedOverrides(const std::array<JointOverride, armJointCount>& newOverrides) {
        std::copy(std::begin(newOverrides), std::end(newOverrides), std::begin(overrides));
    }

    // Overwrite any overrides specified. Any unspecified joint override will keep its current value.
    void applyOverrides(const std::map<ArmJoint, JointOverride>& overridesToApply) {
        auto current = getOrderedOverrides();
        for (auto& i : overridesToApply) {
            current[static_cast<int>(i.first)] = i.second;
        }
        setOrderedOverrides(current);
    }

    void enableShoulderConstraints() { shoulderConstraintsEnabled = true; }
    void disableShoulderConstraints() { shoulderConstraintsEnabled = false; }
    bool areShoulderConstraintsEnabled() { return shoulderConstraintsEnabled; }

    void enableGravityTorque() { gravityTorqueDisabled = false; }
    void disableGravityTorque() { gravityTorqueDisabled = true; }
    bool isGravityTorqueDisabled() { return gravityTorqueDisabled; }
    
    void enableSHRTorque() { SHRTorqueDisabled = false; }
    void disableSHRTorque() { SHRTorqueDisabled = true; }
    bool isSHRTorqueDisabled() { return SHRTorqueDisabled; }
};

} // namespace harmony

#endif