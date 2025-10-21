/**
 * @file perturbTest
 * @author Rhet O. Hailey (rhet.hailey@auburn.edu)
 * @brief test torque speeds
 * @version 0.1
 * @date 2024-07-2
 *
 * @copyright Copyright (c) 2024
 *
 */

/******************************************************************************************
 * INCLUDES
 *****************************************************************************************/
#include "research_interface.h"
#include <Eigen/Dense>
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#include "torqueWrapper.h"

#define PI 3.141592653
#define DEG_2_RAD PI / 180
#define nCols 2 * harmony::armJointCount + 1
// #define nCols 15
#define N_JOINTS 7

#define s400Stiffness_Nm_p_rad 50.0 // desired joint stiffness for series 400 [shoulder] motors(max is 50)
#define s600Stiffness_Nm_p_rad 30.0 // desired joint stiffness for series 600 [elbow] motors (max is 30)
#define s700Stiffness_Nm_p_rad 3.0 // desired joint stiffness for series 700 [wrist] motors(max is 3)

#ifdef WRIST_MODS
#define WD_SCALING 1.0
#define WF_SCALING 0.33
#endif


/******************************************************************************************
 * MAIN
 *****************************************************************************************/
int main() {

    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("log/harmony_log.csv", 80000, 10); // Create the 1st appender.
    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender; // Create the 2nd appender.


    plog::init(plog::info, &fileAppender).addAppender(&consoleAppender); // Initialize the logger with the both appenders.

    // initialize info as harmony
    harmony::ResearchInterface* info = new harmony::ResearchInterface;
    harmony_robot robot; // this is a struct
    
    // stop if failed
    if (!info->init()) {
        std::cerr << "Failed to initialize Research Interface" << std::endl;
        return -1;
    }
    char side = 'B';
    robot.set_ordered_states(info);
    Eigen::VectorXd rightTorque = robot.get_torque_Nm(side);
    Eigen::VectorXd rightPos = robot.get_position_rads(side);

    // initialize arm controllers
    auto left = info->makeLeftArmController();
    auto right = info->makeRightArmController();

    // stop if failed
    if (!left->init() || !right->init()) {
        std::cerr << "Failed to Arm Controllers" << std::endl;
        return -1;
    }

    // user inputs torques
    // *** TODO *** //

    // hard-coded bullshit
    std::array<double, nCols> hardCode;
    for (int i = 0; i < nCols; i++) {
        /* code */
        hardCode[i] = 0;

        // wrists
        if (i == 6) { hardCode[i] = 10*1000; }
        if (i == 13) { hardCode[i] = 10*1000; }

        std::cout << hardCode[i] << ", ";
    }
    std::cout << "\n";

    // rename it in worst way
    std::array<double, nCols> desiredTorque;
    desiredTorque = hardCode;

    // convert torques to override
    std::cout << "torque 2 override\n";
    auto overrides = convert_torque_to_override(desiredTorque, info, side); // hard coded bs

    // set sided overrides
    left->setJointsOverride(overrides.leftOverrides);
    right->setJointsOverride(overrides.rightOverrides);

    // timer inits
    int T_ms = 4000; // 4s
    int writeTime = 5;

    auto start = std::chrono::high_resolution_clock::now();
    auto end = start + std::chrono::milliseconds(static_cast<long long int>(T_ms));
    auto now = std::chrono::high_resolution_clock::now();
    auto nextWrite = start + std::chrono::milliseconds(static_cast<long long int>(writeTime));

    // do this for x(s)
    while (now <= end) {
        now = std::chrono::high_resolution_clock::now();
        
        // this changes override
        overrides = convert_torque_to_override(hardCode, info, side);
        
        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);

        if (now >= nextWrite) {
            // updates information?
            robot.set_ordered_states(info); // this throws error? why necessary?

            nextWrite = now + std::chrono::milliseconds(static_cast<long long int>(writeTime));
        }
    }

    // quit overrides
    left->removeOverride();
    right->removeOverride();

    // exit
    delete info;
    return 0;
}
