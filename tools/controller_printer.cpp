#include "research_interface.h"
#include <Eigen/Geometry>
#include <iostream>

void printPose(harmony::Pose pose) {
    std::cout << "Desired pose:" << std::endl;
    std::cout << "position (mm): (" << pose.position_mm.x << ", " << pose.position_mm.y << ", " << pose.position_mm.z
              << ")" << std::endl;
    std::cout << "orientation: [x: " << pose.orientation.x << " y: " << pose.orientation.y
              << " z: " << pose.orientation.z << " w: " << pose.orientation.w << "]    ";

    Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    auto angles_deg = quaternion.toRotationMatrix().eulerAngles(0, 1, 2) * 180.0 / 3.1415926536;
    std::cout << "[roll: " << angles_deg[0] << "° pitch: " << angles_deg[1] << "° yaw: " << angles_deg[2] << "°]"
              << std::endl;
    std::cout << std::endl;
}

void printJointsOverride(std::array<harmony::JointOverride, harmony::armJointCount> overrides) {
    for (size_t i = 0; i < overrides.size(); ++i) {

            std::cout << "Joint " << i << ":"
                    << " Desired position (rad): " << overrides[i].desiredPosition_rad
                    << " Desired stiffness (Nm/rad): " << overrides[i].desiredStiffness_Nm_per_rad
                    << " Desired Torque (Nm): " << overrides[i].desiredTorque_Nm << std::endl;
    }
}

void printOverrides(std::unique_ptr<harmony::ArmController>& controller) {
    switch (controller->getMode()) {
        case harmony::ArmController::Mode::harmony:
            std::cout << "Mode: harmony";
            break;
        case harmony::ArmController::Mode::jointsOverride:
            std::cout << "Mode: joints override" << std::endl;
            std::string shoulderConstraintsEnabledString =
                controller->getArmJointsOverride()->areShoulderConstraintsEnabled() ? "true" : "false";
            std::cout << "Shoulder constraints enabled: " << shoulderConstraintsEnabledString << std::endl;
            std::string gravityTorqueDisabledString =
                controller->getArmJointsOverride()->isGravityTorqueDisabled() ? "true" : "false";
            std::cout << "Gravity torque disabled: " << gravityTorqueDisabledString << std::endl;
            std::string SHRTorqueDisabledString =
                controller->getArmJointsOverride()->isSHRTorqueDisabled() ? "true" : "false";
            std::cout << "SHR torque disabled: " << SHRTorqueDisabledString << std::endl;
            printJointsOverride(controller->getArmJointsOverride()->getOrderedOverrides());
            break;
    }
    std::cout << std::endl;
}

int main() {
    auto right = harmony::ResearchInterface::makeRightArmController();
    if (!right->init()) {
        std::cerr << "Failed to initialize right controller" << std::endl;
        exit(-1);
    }
    std::cout << "Right arm controller:" << std::endl;
    printOverrides(right);

    std::cout << "===============" << std::endl;

    auto left = harmony::ResearchInterface::makeLeftArmController();
    if (!left->init()) {
        std::cerr << "Failed to initialize left controller" << std::endl;
        exit(-1);
    }
    std::cout << "Left arm controller:" << std::endl;
    printOverrides(left);

}