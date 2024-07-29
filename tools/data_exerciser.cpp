/**
 * @file exercise_reader.cpp
 * @brief Read a log file and write it back to Harmony!
 * @version 0.1
 */

/******************************************************************************************
 * INCLUDES
 *****************************************************************************************/
#include "research_interface.h"
#include <array>
#include <chrono>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

#define PI 3.141592653
#define DEG_2_RAD PI / 180

#define s400Stiffness_Nm_p_rad 50.0 // desired joint stiffness for series 400 [shoulder] motors(max is 50)
#define s600Stiffness_Nm_p_rad 30.0 // desired joint stiffness for series 600 [elbow] motors (max is 30)
#define s700Stiffness_Nm_p_rad  3.0 // desired joint stiffness for series 700 [wrist] motors(max is 3)

#define nCols 2 * harmony::armJointCount + 1 // number of columns in dataset

/******************************************************************************************
 * Structs
 *****************************************************************************************/
struct AllArmsOverrides {
    harmony::ArmJointsOverride leftOverrides;
    harmony::ArmJointsOverride rightOverrides;
};

/******************************************************************************************
 * FUNCTIONS
 *****************************************************************************************/

/**
 * @brief Print the use case for this script
 *
 * @return int
 */
int printUses() {
    std::cout << "Uses: \n"
              << "harmony_exerciser <logfile-prefix>\n"
              << "\tWhere the log file is: log/<prefix>_log.txt" << std::endl;
    return -1;
}

/**
 * @brief Prints the start script before writing to harmony
 *
 * @param fs sample frequency from logfile
 */
void printStart(double fs) {
    // std::cout << "stiffness : " << stiffness_Nm_p_rad << " Nm per rad\n";
    std::cout << "fs : " << fs << " Hz\n";
    std::cout << "Starting in";
    std::cout.flush();

    for (int i = 3; i >= 1; i--) {
        std::cout << "..." << i;
        std::cout.flush();
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "\n--> LETS EXERCISE <--\n";
}

/**
 * @brief From a string of data, split into an array
 * Uses ', ' as deliminator
 * @param line data line from logfile
 * @return std::array<double, nCols> dataLine as an array of doubles
 */
std::array<double, nCols> parseLineOfData(std::string line) {
    std::array<double, nCols> dataLine;

    std::string del = ", ";
    int start = 0;
    int end = line.find(del);
    for (int i = 0; i < nCols; i++) {
        dataLine[i] = std::atof(line.substr(start, end - start).c_str()) * DEG_2_RAD;

        start = end + del.size();
        end = line.find(del, start);
    }

    return dataLine;
}

/**
 * @brief IIR low-pass-filter of data
 *
 * @param data current dataset
 * @param prevData previous dataset
 * @param fs recording frequency
 */
void IIRfilter(std::array<double, nCols>* data, std::array<double, nCols> prevData, double fs) {
    double fc = 0.5 * fs;
    double Ts = 1 / fs;

    double alpha = 2 * PI * Ts * fc / (2 * PI * Ts * fc + 1);

    std::array<double, nCols> filteredData{ prevData };
    for (int i = 1; i < nCols; i++) {
        filteredData[i] = alpha * data->at(i) + (1 - alpha) * prevData[i];
    }
    *data = filteredData;
}

/**
 * @brief returns the desired joint stiffness based on the actuator type
 * Joints <5 are all series 400, joint 5 is the elbow series 500, and
 * joint 6 is the wrist series 700 joint.
 * @param joint_idx current index for joint
 * @return double the desired joint stiffness
 */
double jointStiffness(int joint_idx) {
    double stiffness;
    harmony::ArmJoint joint = harmony::ArmJoint(joint_idx);

    switch (joint) {
        case harmony::ArmJoint::elbowFlexion:
            stiffness = s600Stiffness_Nm_p_rad;
            break;
        case harmony::ArmJoint::wristPronation:
            stiffness = s700Stiffness_Nm_p_rad;
            break;
        default:
            stiffness = s400Stiffness_Nm_p_rad;
            break;
    }

    return stiffness;
}


/**
 * @brief returns the desired joint stiffness based on the actuator type
 * Joints <5 are all series 400, joint 5 is the elbow series 500, and
 * joint 6 is the wrist series 700 joint.
 * @param joint_idx current index for joint
 * @param scaling is used to indirectly control the stiffnesses
 * @return double the desired joint stiffness
 */
double jointStiffness(int joint_idx, double scaling) {
    double stiffness;
    harmony::ArmJoint joint = harmony::ArmJoint(joint_idx);

    switch (joint) {
        case harmony::ArmJoint::elbowFlexion:
            stiffness = s600Stiffness_Nm_p_rad * scaling;
            break;
        case harmony::ArmJoint::wristPronation:
            stiffness = s700Stiffness_Nm_p_rad * scaling;
            break;
        default:
            stiffness = s400Stiffness_Nm_p_rad;
            break;
    }

    return stiffness;
}

/**
 * @brief Takes a data array and converts it to override format
 *
 * @param data read data line from log file, parsed to array
 * @return AllArmsOverrides studt containing left and right overrides
 */
AllArmsOverrides data2override(std::array<double, nCols> data) {
    std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;
    for (int i = 0; i < harmony::armJointCount; i++) {
        leftOverrides[i] = {data[i + 1], jointStiffness(i)};
        rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i)};
    }

    return {harmony::ArmJointsOverride(leftOverrides),
            harmony::ArmJointsOverride(rightOverrides)
            };
}

/**
 * @brief Takes a data array and converts it to override format
 *
 * @param data read data line from log file, parsed to array
 * @param scaling indirectly controlls impedence values
 * @return AllArmsOverrides studt containing left and right overrides
 */
AllArmsOverrides data2override(std::array<double, nCols> data, double scaling) {
    std::array<harmony::JointOverride, harmony::armJointCount> leftOverrides;
    std::array<harmony::JointOverride, harmony::armJointCount> rightOverrides;
    for (int i = 0; i < harmony::armJointCount; i++) {
        leftOverrides[i] = {data[i + 1], jointStiffness(i, scaling)};
        rightOverrides[i] = {data[i + harmony::armJointCount + 1], jointStiffness(i, scaling)};
    }

    return {harmony::ArmJointsOverride(leftOverrides),
            harmony::ArmJointsOverride(rightOverrides)
            };
}


/**
 * @brief Read info for current arm positions and return the posisitons as Joint Overrides
 * 
 * @param info pointer to research interface
 * @return AllArmsOverrides object holding left and right
 */
std::array<double, nCols> getCurrentArmPositionsAsDataLine(harmony::ResearchInterface* info) {
    auto leftStates = info->joints().leftArm.getOrderedStates();
    auto rightStates = info->joints().rightArm.getOrderedStates();

    std::array<double, nCols> data;

    data[0] = 0.0;

    for (int i = 0; i < harmony::armJointCount; i++) { data[i + 1] = leftStates[i].position_rad; }
    for (int i = 0; i < harmony::armJointCount; i++) { data[i + harmony::armJointCount + 1] = rightStates[i].position_rad; }

    return data;
}

/**
 * @brief Given an initial and target position, interpolate between the two
 * Moves each joint (linearly) towards the traget posiiton with the iter representing the
 * current step taken out of nSteps. This function returns AllArmsOverrides to be pushed to
 * Harmony
 * @param initialOverride Initial position as AllArmsOverrides
 * @param targetOverride Target Position as AllArmsOverrides
 * @param iter iteration of interpolation
 * @param nSteps number of steps to take between initial and target positions 
 * @return AllArmsOverrides the override command per the _iter_ step
 */
std::array<double, nCols> step2targetPosition(  std::array<double, nCols> start,
                                                std::array<double, nCols> finish,
                                                int iter, int nSteps ) {

    std::array<double, nCols> step;
    step[0] = 0.;

    for (int i = 1; i < nCols; i++) {
        step[i] = start[i] + (finish[i] - start[i]) * iter / nSteps;
    }
    return step;
    
}



/******************************************************************************************
 * Main
 *****************************************************************************************/

int main(int argc, const char** argv) {
    /*-------- Parse Args --------*/
    if (argc < 2) { return printUses(); }

    /*-------- Init logfile --------*/
    std::string prefix(argv[1]);
    std::string filename = "./log/" + prefix + "_log.txt";
    std::ifstream logFile(filename);

    if (!logFile.is_open()) {
        std::cerr << "ERROR, could not find file:\n\t" << filename << std::endl;
        return -1;
    }
    std::cout << "Found file: '" << filename << "'\n";

    std::string line; // read line from log file
    double fs; // recording frequency
    uint T_ms; // recording time step
    if (std::getline(logFile, line)) {
        fs = atof(line.substr(5, line.size() - 5).c_str()); // First line contains fs
        T_ms = uint(1000 / fs);
    }
    std::getline(logFile, line); // second line is headers
    std::getline(logFile, line); // start from third line
    std::array<double, nCols> exerciseStartPos = parseLineOfData(line); // holds previous data for filter

    /*--------- Init Research Interface --------*/
    harmony::ResearchInterface info;
    if(!info.init()) {
        std::cerr << "Failed to initialize Research Interface" << std::endl;
        return -1;
    }

    auto left = info.makeLeftArmController(); // left arm controller
    auto right = info.makeRightArmController(); // right arm controller
    if (!left->init() || !right->init()) {
        std::cerr << "Failed to initialize Arm Controllers" << std::endl;
        return -1;
    }


    /*--------- Scale Up Impedence Control --------*/
    std::cout << "Scaling Up Impedence Control Values [4s]";
    std::cout.flush();

    int bufferTime_s = 4;
    int nSteps = bufferTime_s * fs;
    auto robotStartPosition = getCurrentArmPositionsAsDataLine(&info);

    for (int i=0; i<=nSteps; i++) {
        robotStartPosition = getCurrentArmPositionsAsDataLine(&info);
        auto overrides = data2override(robotStartPosition, i/nSteps);

        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);

        if (i * T_ms % 1000 == 0) {
            std::cout << ".";
            std::cout.flush();
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
    }
    
    std::cout << "DONE\n"; 

    /*--------- Move Harmony to start position --------*/
    std::cout << "Moving Harmony to starting position [10s]";
    std::cout.flush();
    bufferTime_s = 10;
    nSteps = bufferTime_s * fs;

    std::array<double, nCols> data;
    std::array<double, nCols> prevData = robotStartPosition;
    for (int i=0; i<=nSteps; i++) {
        data = step2targetPosition(robotStartPosition, exerciseStartPos , i, nSteps);
        // IIRfilter(&data, prevData, fs);

        auto overrides = data2override(data);

        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);

        if (i * T_ms % 1000 == 0) {
            std::cout << ".";
            std::cout.flush();
        }    
        std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
        prevData = data;
    }

    std::cout << "DONE\n";

    /*--------- Wait to start and then begin exercise --------*/
    printStart(fs);
    while (std::getline(logFile, line)) {
        data = parseLineOfData(line); // array of timestamp and joint positions
        IIRfilter(&data, prevData, fs);

        auto overrides = data2override(data); // Holds overrides for both sides

        left->setJointsOverride(overrides.leftOverrides);
        right->setJointsOverride(overrides.rightOverrides);

        prevData = data;
        std::this_thread::sleep_for(std::chrono::milliseconds(T_ms));
    }
    std::cout << "WE DID IT!" << std::endl;

    /*--------- Close out --------*/
    logFile.close();
    left->removeOverride();
    right->removeOverride();

    return 0;
}