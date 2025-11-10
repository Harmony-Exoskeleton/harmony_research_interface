#include "shared_memory.h"
#include "shared_memory_manager.h"
#include "arm_controller.h"
#include <iostream>
#include <cstring>

int main() {
    std::cout << "Resetting Harmony shared memory..." << std::endl;

    // Initialize shared memory managers (as non-owners so we don't destroy them)
    harmony::SharedMemoryManager<harmony::DataFromHarmony> harmonyStateManager(harmony::dataFromHarmonyMemId, false);
    harmony::SharedMemoryManager<harmony::DataToHarmony> harmonyCommandManager(harmony::dataToHarmonyMemId, false);
    harmony::SharedMemoryManager<harmony::ArmControllerState> leftControllerStateManager(
        harmony::leftArmControllerMemId, false);
    harmony::SharedMemoryManager<harmony::ArmControllerState> rightControllerStateManager(
        harmony::rightArmControllerMemId, false);

    // Initialize all managers
    bool success = harmonyStateManager.init() && 
                   harmonyCommandManager.init() && 
                   leftControllerStateManager.init() && 
                   rightControllerStateManager.init();

    if (!success) {
        std::cerr << "Failed to initialize shared memory managers" << std::endl;
        return -1;
    }

    // Reset DataFromHarmony - zero out all data
    std::memset(harmonyStateManager.data, 0, sizeof(harmony::DataFromHarmony));
    std::cout << "  - Reset DataFromHarmony (joint states, sizes, poses, digital inputs)" << std::endl;

    // Reset DataToHarmony - zero out all data
    std::memset(harmonyCommandManager.data, 0, sizeof(harmony::DataToHarmony));
    std::cout << "  - Reset DataToHarmony (digital outputs)" << std::endl;

    // Reset left ArmControllerState to defaults
    harmony::ArmControllerState defaultControllerState;
    defaultControllerState.mode = harmony::ArmController::Mode::harmony;
    defaultControllerState.shoulderConstraintsEnabled = true;
    defaultControllerState.gravityTorqueDisabled = false;
    defaultControllerState.SHRTorqueDisabled = false;
    // Zero out joint overrides
    std::memset(defaultControllerState.jointsOverride, 0, 
               sizeof(harmony::JointOverride) * harmony::armJointCount);
    
    *leftControllerStateManager.data = defaultControllerState;
    std::cout << "  - Reset left ArmControllerState (mode=harmony, constraints=enabled)" << std::endl;

    // Reset right ArmControllerState to defaults
    *rightControllerStateManager.data = defaultControllerState;
    std::cout << "  - Reset right ArmControllerState (mode=harmony, constraints=enabled)" << std::endl;

    std::cout << "Shared memory reset complete!" << std::endl;
    return 0;
}

