/** @file
 * @brief Declares Harmony SHR's external research interface.
 *
 */

#ifndef HARMONY_RESEARCH_INTERFACE_H
#define HARMONY_RESEARCH_INTERFACE_H

#include "arm_controller.h"
#include "joint_states.h"
#include "pose.h"
#include "shared_memory_manager.h"
#include "sizes.h"
#include <array>

namespace harmony {

struct DataFromHarmony; // forward declare
struct DataToHarmony; // forward declare

class ResearchInterface {
public:
    ResearchInterface();
    ResearchInterface(const ResearchInterface&) = delete;
    ResearchInterface& operator=(const ResearchInterface&) = delete;

    bool init();

    JointStates joints() const;
    Sizes sizes() const;
    Poses poses() const;

    std::array<bool, 2> readDigitalInput() const;

    void writeDigitalOutput(std::array<bool, 2> values);
    void writeDigitalOutput0(bool value);
    void writeDigitalOutput1(bool value);

    static std::unique_ptr<ArmController> makeLeftArmController();
    static std::unique_ptr<ArmController> makeRightArmController();

private:
    SharedMemoryManager<DataFromHarmony> fromHarmonyManager;
    SharedMemoryManager<DataToHarmony> toHarmonyManager;
};

} // namespace harmony
#endif