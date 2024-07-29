#include "shared_memory.h"
#include "shared_memory_manager.h"
#include <iostream>

int main() {
    harmony::SharedMemoryManager<harmony::DataFromHarmony> harmonyStateManager(harmony::dataFromHarmonyMemId, true);
    harmony::SharedMemoryManager<harmony::ArmControllerState> leftControllerStateManager(
        harmony::leftArmControllerMemId, true);
    harmony::SharedMemoryManager<harmony::ArmControllerState> rightControllerStateManager(
        harmony::rightArmControllerMemId, true);

    bool success = harmonyStateManager.init() && leftControllerStateManager.init() && rightControllerStateManager.init();
    if (!success) {
        std::cerr << "Failed to initialize a shared memory manager" << std::endl;
        exit(-1);
    }

    std::string str;
    do {
        std::cout << "Harmony shared memory has been initialized. Hit Ctrl-D to end." << std::endl;
    } while (std::cin >> str);
}
