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

using namespace std;
using namespace rosbridge2cpp;
using namespace harmony;

/******************************************************************************************
 * DEFINES
 *****************************************************************************************/

#define DEFAULT_LOOP_TIME_MS 10
#define ROSBRIDGE_DEFAULT_PORT 9090
#define ROSBRIDGE_DEFAULT_HOST "127.0.0.1"

/******************************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************************/

static ResearchInterface* g_research_interface = nullptr;
static ROSBridge* g_ros_bridge = nullptr;
static ROSTopic* g_joint_state_pub = nullptr;
static ROSTopic* g_left_pose_pub = nullptr;
static ROSTopic* g_right_pose_pub = nullptr;
static ROSTopic* g_left_sizes_pub = nullptr;
static ROSTopic* g_right_sizes_pub = nullptr;
static ROSTopic* g_left_arm_torque_sub = nullptr;
static ROSTopic* g_right_arm_torque_sub = nullptr;
static ArmController* g_left_arm_controller = nullptr;
static ArmController* g_right_arm_controller = nullptr;

/******************************************************************************************
 * ROS CALLBACKS
 *****************************************************************************************/

void connection_error_handler(TransportError err) {
    if (err == TransportError::R2C_CONNECTION_CLOSED) {
        PLOGE << "ROSBridge2cpp connection closed - You should reinit ROSBridge2cpp";
    }
    if (err == TransportError::R2C_SOCKET_ERROR) {
        PLOGE << "Error on ROSBridge2cpp Socket - You should reinit ROSBridge2cpp";
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

    // Header
    rapidjson::Value header(rapidjson::kObjectType);
    rapidjson::Value stamp(rapidjson::kObjectType);
    stamp.AddMember("sec", 0, allocator);
    stamp.AddMember("nanosec", 0, allocator);
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

    g_joint_state_pub->Publish(msg);
}

/******************************************************************************************
 * MAIN
 *****************************************************************************************/
int main(int argc, char* argv[]) {

    // Initialize logging
    static plog::RollingFileAppender<plog::CsvFormatter> fileAppender("log/harmony_ros_interface_log.csv", 80000, 10);
    plog::init(plog::info, &fileAppender);

    static plog::ColorConsoleAppender<plog::TxtFormatter> consoleAppender;
    plog::get()->addAppender(&consoleAppender);

    PLOGI << "Harmony ROS Interface starting...";

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
    g_ros_bridge = &ros_bridge;

    // Get ROS bridge connection parameters from environment or use defaults
    const char* ros_host = getenv("ROSBRIDGE_HOST");
    const char* ros_port_str = getenv("ROSBRIDGE_PORT");
    int ros_port = ROSBRIDGE_DEFAULT_PORT;
    
    if (ros_port_str) {
        ros_port = std::stoi(ros_port_str);
    }

    std::string host = ros_host ? ros_host : ROSBRIDGE_DEFAULT_HOST;

    PLOGI << "Attempting to connect to ROSBridge server at " << host << ":" << ros_port;
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

    auto loop_period = std::chrono::milliseconds(DEFAULT_LOOP_TIME_MS);
    auto next_cycle = std::chrono::steady_clock::now();

    while (true) {

        // Publish all data (following pattern from data_logger and value_printer)
        publish_joint_states();

        // Sleep until next cycle
        next_cycle += loop_period;
        std::this_thread::sleep_until(next_cycle);
    }

    return 0;
}

