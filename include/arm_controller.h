#ifndef HARMONY_ARM_CONTROLLER_H
#define HARMONY_ARM_CONTROLLER_H

#include "overrides.h"
#include "shared_memory_manager.h"
#include <memory>

namespace harmony {
struct ArmControllerState; // forward declare

class ArmController {
public:
    enum class Mode {
        harmony = 0,
        jointsOverride = 1,
    };

    explicit ArmController(int memId);
    ArmController(const ArmController&) = delete;
    ArmController& operator=(const ArmController&) = delete;

    bool init();

    void removeOverride();
    void setJointsOverride(ArmJointsOverride override);

    Mode getMode() const;

    // will return null if mode is not jointsOverride
    std::unique_ptr<ArmJointsOverride> getArmJointsOverride() const;

private:
    SharedMemoryManager<ArmControllerState> manager;
};

} // namespace harmony

#endif