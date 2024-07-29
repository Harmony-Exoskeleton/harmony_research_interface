/**
 * @file harmony_data_logger.cpp
 * @brief Logs joint positions using the Harmony Research Interface!
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
#define RAD_2_DEG 180 / PI

/******************************************************************************************
 * FUNCTIONS
 *****************************************************************************************/

/**
 * @brief Prints the Uses for this script
 *
 * @return int
 */
int printUses() {
    std::cout << "Uses: \n"
              << "harmony_logger <logfile-prefix> <(Optional) framerate>"
              << "\tWhere the framerate defaults to 200 Hz" << std::endl;
    return -1;
}

/**
 * @brief Writes the header for the log file
 * The first line of the log file is the sampling frequency,
 * the second line is the column names
 * the nth line is the nth recorded joint position vector with
 * respect to time
 * @param logFile the ofstream object for the file to be written
 * @param fs the sampling frequency in Hz
 */
void printLogHeader(std::ofstream* logFile, double fs) {
    *logFile << "fs = " << fs << "\nt, ";
    for (int i=0; i<harmony::armJointCount; i++){ *logFile << "left_j" << i << ", "; }
    for (int i=0; i<harmony::armJointCount; i++){ *logFile << "right_j" << i << ", "; }
    *logFile << std::endl;
}

/**
 * @brief Convert a given file prefix to a log filename, including path.
 *  Makes all files end in a '-log.txt'
 * @param filePrefix the desired file prefix
 * @return std::string logfilename
 */
std::string filepath(std::string filePrefix) {
    return "./log/" + filePrefix + "_log.txt";
}

/**
 * @brief Locks the user in place to start recording
 *
 * @return true if the user hits [return]
 * @return false if the user hits [q]
 */
bool waitToStart() {
    std::string usr_input;

    // lambda function to check if valid response
    auto valid_response = [&]() -> bool { return usr_input.length() == 0 || usr_input == "q"; };

    do {
        std::cout << "Enter [return] to start recording or [q] to quit. ";
        std::getline(std::cin, usr_input);
    } while (!valid_response());

    bool startRecording = usr_input.length() == 0;
    if (!startRecording) { std::cout << "Exiting..." << std::endl; }

    return startRecording;
}

/**
 * @brief Log the data to a log file
 *
 * @param logfile is the ofstream object for logging
 * @param states is the read joint states of the object
 */
void logData(std::ofstream* logfile, std::array<harmony::JointState, harmony::armJointCount> states) {
    for (int i = 0; i < harmony::armJointCount; i++) {
        *logfile << states[i].position_rad * RAD_2_DEG << ", ";
    }
}

/**
 * @brief Thread function to stop while loop on usr command [Ctrl-D]
 *
 * @param spin is the while loop flag from main
 */
void loopSpin(bool* spin) {
    std::string str;

    while (*spin) {
        *spin = bool(std::cin >> str);
    }
}

/******************************************************************************************
 * MAIN
 *****************************************************************************************/

int main(int argc, const char** argv) {
    // Parse args
    if (argc < 2) { return printUses(); }

    double fs = 200; // Frame rate in hz
    if (argc == 3) { fs = double(*argv[2]); }

    // init research interface
    harmony::ResearchInterface info;
    if (!info.init()) {
        std::cerr << "ERROR: Research Interface failed to initialize!" << std::endl;
        return -1;
    }

    // lock until user before start of recording
    if (!waitToStart()) { return -1; }

    // open logFile and set header
    std::string filePrefix(argv[1]); // file prefix specified by user
    std::ofstream logFile(filepath(filePrefix), std::ios::out); // ofstream object for writing
    printLogHeader(&logFile, fs);

    // setup loop vars
    double T_s = 1 / fs; // Time_step in seconds
    char logIndicator[4]{'-', '\\', '|', '/'}; // indicator for recording... adds some flair
    int i = 0; // loop iteration count
    bool spin = true; // while loop flag

    // record until [Ctrl-D] is pressed
    std::thread spinThread(loopSpin, &spin); // thread to stop the while loop
    std::cout << "Enter [Ctrl-D] to stop recording.\n";
    while (spin) {
        logFile << i * T_s << ", ";
        logData(&logFile, info.joints().leftArm.getOrderedStates());
        logData(&logFile, info.joints().rightArm.getOrderedStates());

        logFile << "\n";

        std::cout << "Recording " << logIndicator[i % 4] << '\r';
        std::cout.flush();

        std::this_thread::sleep_for(std::chrono::milliseconds(uint(1000 * T_s)));
        ++i;
    }

    spinThread.join();
    logFile.close();
    std::cout << "Recording DONE\nRecorded to " << filepath(filePrefix) << std::endl;
    return 1;
}