/**
 * @file harmony_ros_interface
 * @author Stefano Dalla Gasperina (stefano.dallagasperina@austin.utexas.edu)
 * @brief ROS interface for Harmony research interface using rosbridge
 * @version v1.0
 * @date 2025-11-06
 *
 * @copyright Copyright (c) 2025
 *
 */

/******************************************************************************************
 * INCLUDES
 *****************************************************************************************/
#include "research_interface.h"
#include "plog/Log.h"
#include "plog/Appenders/ColorConsoleAppender.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "types.h"
#include "client/socket_websocket_connection.h"
#include "rapidjson/document.h"

#include <thread>
#include <chrono>
#include <string>
#include <cstdlib>
#include <iomanip>
#include <iostream>

using namespace std;
using namespace rosbridge2cpp;
using namespace harmony;

/******************************************************************************************
 * DEFINES
 *****************************************************************************************/

#define DEFAULT_LOOP_FREQUENCY_HZ 100
#define LOG_FREQUENCY_HZ 1  // Fixed at 1 Hz (log once per second)
#define ROSBRIDGE_DEFAULT_PORT 9090
#define ROSBRIDGE_DEFAULT_HOST "127.0.0.1"

/******************************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************************/

static ResearchInterface* g_research_interface = nullptr;
static ROSTopic* g_joint_state_pub = nullptr;

/******************************************************************************************
 * ROS CALLBACKS
 *****************************************************************************************/

void connection_error_handler(TransportError err) {
    if (err == TransportError::R2C_CONNECTION_CLOSED) {
        PLOGE << "ROSBridge connection closed - reconnecting...";
    }
    if (err == TransportError::R2C_SOCKET_ERROR) {
        PLOGE << "ROSBridge socket error - reconnecting...";
    }
}

/******************************************************************************************
 * ROS PUBLISHING
 *****************************************************************************************/

void publish_joint_states() {
    if (!g_research_interface || !g_joint_state_pub) {
        return;
    }

    auto joint_states = g_research_interface->joints();
    auto left_states = joint_states.leftArm.getOrderedStates();
    auto right_states = joint_states.rightArm.getOrderedStates();

    // Create joint state message
    // Format: sensor_msgs/JointState
    rapidjson::Document msg;
    msg.SetObject();
    auto& allocator = msg.GetAllocator();

    // Header with timestamp
    auto now = std::chrono::system_clock::now();
    auto duration = now.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nanoseconds = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - seconds);
    
    rapidjson::Value header(rapidjson::kObjectType);
    rapidjson::Value stamp(rapidjson::kObjectType);
    stamp.AddMember("sec", static_cast<int64_t>(seconds.count()), allocator);
    stamp.AddMember("nanosec", static_cast<uint32_t>(nanoseconds.count()), allocator);
    header.AddMember("stamp", stamp, allocator);
    header.AddMember("frame_id", rapidjson::Value("", allocator), allocator);
    msg.AddMember("header", header, allocator);

    // Joint names
    rapidjson::Value names(rapidjson::kArrayType);
    const char* joint_names[] = {
        "left_scapular_elevation", "left_scapular_protraction", "left_shoulder_abduction",
        "left_shoulder_rotation", "left_shoulder_flexion", "left_elbow_flexion", "left_wrist_pronation",
        "right_scapular_elevation", "right_scapular_protraction", "right_shoulder_abduction",
        "right_shoulder_rotation", "right_shoulder_flexion", "right_elbow_flexion", "right_wrist_pronation"
    };
    for (const char* name : joint_names) {
        names.PushBack(rapidjson::Value(name, allocator), allocator);
    }
    msg.AddMember("name", names, allocator);

    // Positions
    rapidjson::Value positions(rapidjson::kArrayType);
    for (const auto& state : left_states) {
        positions.PushBack(state.position_rad, allocator);
    }
    for (const auto& state : right_states) {
        positions.PushBack(state.position_rad, allocator);
    }
    msg.AddMember("position", positions, allocator);

    // Velocities (not available, set to zero)
    rapidjson::Value velocities(rapidjson::kArrayType);
    for (size_t i = 0; i < 2 * armJointCount; ++i) {
        velocities.PushBack(0.0, allocator);
    }
    msg.AddMember("velocity", velocities, allocator);

    // Efforts (torques)
    rapidjson::Value efforts(rapidjson::kArrayType);
    for (const auto& state : left_states) {
        efforts.PushBack(state.torque_Nm, allocator);
    }
    for (const auto& state : right_states) {
        efforts.PushBack(state.torque_Nm, allocator);
    }
    msg.AddMember("effort", efforts, allocator);

    // Publish a message
    g_joint_state_pub->Publish(msg);
}

/******************************************************************************************
 * MAIN
 *****************************************************************************************/

void print_help(const char* program_name) {
    std::cout << "Harmony ROS Interface\n"
              << "ROS interface for Harmony research interface using rosbridge\n\n"
              << "Usage: " << program_name << " [OPTIONS] [LOOP_FREQUENCY_HZ]\n\n"
              << "Arguments:\n"
              << "  LOOP_FREQUENCY_HZ    Publishing loop frequency in Hz (default: " << DEFAULT_LOOP_FREQUENCY_HZ << ")\n\n"
              << "Options:\n"
              << "  -h, --help           Show this help message and exit\n\n"
              << "Note:\n"
              << "  Logging frequency is fixed at " << LOG_FREQUENCY_HZ << " Hz (logs once per second)\n\n"
              << "Environment Variables:\n"
              << "  ROSBRIDGE_HOST       ROSBridge server host (default: 127.0.0.1)\n"
              << "  ROSBRIDGE_PORT       ROSBridge server port (default: 9090)\n\n"
              << "Examples:\n"
              << "  " << program_name << "              # Loop at " << DEFAULT_LOOP_FREQUENCY_HZ << " Hz (default)\n"
              << "  " << program_name << " 200          # Loop at 200 Hz\n"
              << "  " << program_name << " --help       # Show this help message\n"
              << std::endl;
}

int main(int argc, char* argv[]) {

    // Check for help argument
    if (argc >= 2) {
        std::string arg = argv[1];
        if (arg == "--help" || arg == "-h") {
            print_help(argv[0]);
            return 0;
        }
    }

    // Parse command-line arguments
    double loop_frequency_hz = DEFAULT_LOOP_FREQUENCY_HZ;
    if (argc >= 2) {
        try {
            loop_frequency_hz = std::stod(argv[1]);
            if (loop_frequency_hz <= 0) {
                PLOGE << "Loop frequency must be positive. Using default: " << DEFAULT_LOOP_FREQUENCY_HZ;
                loop_frequency_hz = DEFAULT_LOOP_FREQUENCY_HZ;
            }
        } catch (const std::exception& e) {
            PLOGE << "Invalid loop frequency argument. Using default: " << DEFAULT_LOOP_FREQUENCY_HZ;
            loop_frequency_hz = DEFAULT_LOOP_FREQUENCY_HZ;
        }
    }

    // Initialize logging
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("log/harmony_ros_interface_log.csv", 80000, 10);
    plog::init(plog::info, &fileAppender);

    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::get()->addAppender(&consoleAppender);

    PLOGI << "Harmony ROS Interface starting...";
    PLOGI << "Loop frequency: " << loop_frequency_hz << " Hz";
    PLOGI << "Log frequency: " << LOG_FREQUENCY_HZ << " Hz (fixed)";

    // Initialize Research Interface
    ResearchInterface research_interface;
    g_research_interface = &research_interface;

    if (!research_interface.init()) {
        PLOGE << "Research Interface failed to initialize!";
        return 1;
    }
    PLOGI << "Research Interface initialized successfully";

    // Initialize ROS Bridge
    SocketWebSocketConnection transport;
    transport.RegisterErrorCallback(connection_error_handler);
    ROSBridge ros_bridge(transport);

    // Get connection parameters from environment or use defaults
    const char* ros_host = getenv("ROSBRIDGE_HOST");
    const char* ros_port_str = getenv("ROSBRIDGE_PORT");
    int ros_port = ros_port_str ? std::stoi(ros_port_str) : ROSBRIDGE_DEFAULT_PORT;
    std::string host = ros_host ? ros_host : ROSBRIDGE_DEFAULT_HOST;

    PLOGI << "Connecting to ROSBridge server at " << host << ":" << ros_port;
    if (!ros_bridge.Init(host.c_str(), ros_port)) {
        PLOGE << "Failed to connect to ROSBridge server!";
        PLOGE << "Please ensure that:";
        PLOGE << "1. ROS/ROS2 is running";
        PLOGE << "2. ROSBridge server is running:";
        PLOGE << "   For ROS2: ros2 run rosbridge_server rosbridge_websocket";
        PLOGE << "   For ROS1: roslaunch rosbridge_server rosbridge_websocket.launch";
        PLOGE << "3. The server is listening on port " << ros_port;
        return 1;
    }
    PLOGI << "Successfully connected to ROSBridge server!";

    // Create ROS topics
    ROSTopic joint_state_pub(ros_bridge, "/harmony/joint_states", "sensor_msgs/JointState");
    g_joint_state_pub = &joint_state_pub;
    PLOGI << "Created joint state publisher: /harmony/joint_states";

    // Main loop setup
    auto loop_period = std::chrono::microseconds(static_cast<int>(1000000.0 / loop_frequency_hz));
    auto next_cycle = std::chrono::steady_clock::now();
    int loop_count = -1;
    auto start_time = std::chrono::steady_clock::now();
    int publishes_per_log = static_cast<int>(loop_frequency_hz / LOG_FREQUENCY_HZ);

    // Main publishing loop
    while (true) {
        publish_joint_states();
        loop_count++;

        // Log at fixed 1 Hz frequency (only when connected)
        if (publishes_per_log > 0 && loop_count % publishes_per_log == 0) {
            // Only log if connected (not reconnecting)
            if (transport.IsConnected()) {
                auto current_time = std::chrono::steady_clock::now();
                auto time_since_start = std::chrono::duration_cast<std::chrono::microseconds>(
                    current_time - start_time).count();
                
                double actual_hz = (time_since_start > 0) ? (loop_count * 1e6 / time_since_start) : 0.0;

                PLOGI << "Last published (count: " << loop_count << ", Hz: " 
                      << std::fixed << std::setprecision(2) << actual_hz << ")";
                
                auto last_msg = g_joint_state_pub->GetLastPublishedMessage();
                if (!last_msg.empty()) {
                    PLOGD << "Last published message: " << last_msg;
                }
            }
        }

        next_cycle += loop_period;
        std::this_thread::sleep_until(next_cycle);
    }

    return 0;
}

