#include "research_interface.h"
#include "shared_memory.h"
#include "shared_memory_manager.h"
#include <Eigen/Geometry>
#include <iostream>

void printStates(std::array<harmony::JointState, harmony::armJointCount> states) {
    for (int i = 0; i < harmony::armJointCount; i++) {
        std::cout << "joint " << i << " position (rad): " << states[i].position_rad
                  << " torque (Nm): " << states[i].torque_Nm << std::endl;
    }
    std::cout << std::endl;
}

void printSizes(std::array<double, harmony::armSizeCount> sizes_mm) {
    for (int i = 0; i < harmony::armSizeCount; i++) {
        std::cout << "size " << i << " (mm): " << sizes_mm[i] << std::endl;
    }
    std::cout << std::endl;
}

void printPose(harmony::Pose pose) {
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

int main() {
    harmony::ResearchInterface info;
    if (!info.init()) {
        std::cerr << "Failed to initialize research interface" << std::endl;
        return -1;
    }

    std::cout << "right joint states:" << std::endl;
    printStates(info.joints().rightArm.getOrderedStates());

    std::cout << "left joint states:" << std::endl;
    printStates(info.joints().leftArm.getOrderedStates());

    std::cout << "right arm sizes:" << std::endl;
    printSizes(info.sizes().rightArm.getOrderedSizes_mm());

    std::cout << "left arm sizes:" << std::endl;
    printSizes(info.sizes().leftArm.getOrderedSizes_mm());

    std::cout << "right end effector pose:" << std::endl;
    printPose(info.poses().rightEndEffector);

    std::cout << "left end effector pose:" << std::endl;
    printPose(info.poses().leftEndEffector);

}