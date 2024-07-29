#include "overrides.h"
#include "research_interface.h"
#include <iostream>
#include <sstream>
#include <vector>

enum class OverrideControlMode{
    impedanceOverride = 0,
    torqueOverride = 1,
};

std::string exeName = "";

void printUsage(std::string errorMessage) {
    if (!errorMessage.empty()) { std::cout << "Error: " << errorMessage << std::endl << std::endl; }

    std::cout << "Usage: " << exeName << " "
              << "side mode [data] diableGravity disableSHR" << std::endl;
    std::cout << "side values: left, right" << std::endl;
    std::cout << "mode values: harmony, impedance, torque" << std::endl;
    std::cout << "if joints is specified, data is a string of 14 whitespace delimited numbers: position stiffness (for "
                 "each of 7 joints)"
              << std::endl;
    std::cout << "if joints is specified, data is a string of 7 whitespace delimited numbers: torque (for "
                 "each of 7 joints)"
              << std::endl;
    std::cout << "diableGravity values: 0, 1" << std::endl;
    std::cout << "disableSHR values: 0, 1" << std::endl;
}

bool isLeftParse(std::string side) {
    if (side == "left") {
        return true;
    } else if (side == "right") {
        return false;
    } else {
        printUsage("side value must be one of: left, right");
        exit(-1);
    }
}

std::array<int, 2> modeParse(std::string mode) {
    if (mode == "harmony") {
        return {static_cast<int>(harmony::ArmController::Mode::harmony), -1};
    } else if (mode == "impedance") {
        return {static_cast<int>(harmony::ArmController::Mode::jointsOverride),
                static_cast<int>(OverrideControlMode::impedanceOverride)};
    } else if (mode == "torque") {
        return {static_cast<int>(harmony::ArmController::Mode::jointsOverride),
                static_cast<int>(OverrideControlMode::torqueOverride)};
    }else {
        printUsage("mode value must be one of harmony, impedance, torque");
        exit(-1);
    }
}

bool disableParse(std::string disableString) {
    if (disableString == "1") {
        return true;
    } else if (disableString == "0") {
        return false;
    } else {
        printUsage("disable value must be one of 0, 1");
        exit(-1);
    }
}

std::vector<double> splitStringToDoubles(const std::string& str) {
    std::vector<double> result;
    std::istringstream iss(str);
    double d;
    while (iss >> d) {
        result.push_back(d);
    }

    return result;
}

harmony::ArmJointsOverride parseImpedanceOverride(std::string data) {
    auto d = splitStringToDoubles(data);
    if (d.size() != 14) {
        printUsage("impedance data must have 14 values");
        exit(-1);
    }
    return harmony::ArmJointsOverride({harmony::JointOverride{d[0], d[1], 0},
        harmony::JointOverride{d[2], d[3], 0},
        harmony::JointOverride{d[4], d[5], 0},
        harmony::JointOverride{d[6], d[7], 0},
        harmony::JointOverride{d[8], d[9], 0},
        harmony::JointOverride{d[10], d[11], 0},
        harmony::JointOverride{d[12], d[13], 0}});
}

harmony::ArmJointsOverride parseTorqueOverride(std::string data) {
    auto d = splitStringToDoubles(data);
    if (d.size() != 7) {
        printUsage("torque data must have 7 values");
        exit(-1);
    }
    return harmony::ArmJointsOverride({harmony::JointOverride{0, 0, d[0]},
        harmony::JointOverride{0, 0, d[1]},
        harmony::JointOverride{0, 0, d[2]},
        harmony::JointOverride{0, 0, d[3]},
        harmony::JointOverride{0, 0, d[4]},
        harmony::JointOverride{0, 0, d[5]},
        harmony::JointOverride{0, 0, d[6]}});
}

int main(int argc, char** argv) {
    exeName = argv[0];

    if (argc < 6) {
        std::string errorMsg = argc == 1 ? "" : "not enough arguments";
        printUsage(errorMsg);
        exit(-1);
    }
    bool isLeft = isLeftParse(argv[1]);
    auto mode = modeParse(argv[2]);
    auto harmonyMode = static_cast<harmony::ArmController::Mode>(mode[0]);
    auto controlMode = static_cast<OverrideControlMode>(mode[1]);
    bool disableGravity = disableParse(argv[4]);
    bool disableSHR = disableParse(argv[5]);

    if (harmonyMode == harmony::ArmController::Mode::jointsOverride && argc < 4) {
        printUsage("data argument is required for joints mode");
        exit(-1);
    }

    harmony::ResearchInterface research;
    if (!research.init()) {
        std::cerr << "Failed to initialize research interface" << std::endl;
        return -1;
    }

    auto controller = isLeft ? research.makeLeftArmController() : research.makeRightArmController();
    if (!controller->init()) {
        std::cerr << "Failed to initialize arm controller" << std::endl;
        return -1;
    }

    if (harmonyMode == harmony::ArmController::Mode::jointsOverride) {
        if (controlMode == OverrideControlMode::impedanceOverride){
            auto override = parseImpedanceOverride(argv[3]);
            if (disableGravity)
                override.disableGravityTorque(); // Enabled by default
            if (disableSHR)
                override.disableSHRTorque(); // Enabled by default
            controller->setJointsOverride(override);
        }
        else if (controlMode == OverrideControlMode::torqueOverride)
        {
            auto override = parseTorqueOverride(argv[3]);
            if (disableGravity)
                override.disableGravityTorque(); // Enabled by default
            if (disableSHR)
                override.disableSHRTorque(); // Enabled by default
            controller->setJointsOverride(override);

        }
    } else { // harmony mode
        controller->removeOverride();
    }
}