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
#include "ros_tf_broadcaster.h"
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

// Base link position in map/world frame (in meters)
// This defines where the robot base_link is located - doesn't have to be at origin
constexpr double BASE_LINK_POSITION_X_M = 0.0;
constexpr double BASE_LINK_POSITION_Y_M = 0.0;
constexpr double BASE_LINK_POSITION_Z_M = 0.8;
constexpr double BASE_LINK_ORIENTATION_QX = 0.0;
constexpr double BASE_LINK_ORIENTATION_QY = 0.0;
constexpr double BASE_LINK_ORIENTATION_QZ = 0.0;
constexpr double BASE_LINK_ORIENTATION_QW = 1.0;

/******************************************************************************************
 * GLOBAL VARIABLES
 *****************************************************************************************/

static ResearchInterface* g_research_interface = nullptr;
static ROSTopic* g_left_joint_state_pub = nullptr;
static ROSTopic* g_right_joint_state_pub = nullptr;
static ROSTFBroadcaster* g_tf_broadcaster = nullptr;

/******************************************************************************************
 * ROS CALLBACKS
 *****************************************************************************************/

void connection_error_handler(TransportError err) {
    if (err == TransportError::R2C_CONNECTION_CLOSED) {
        PLOGW << "ROSBridge connection closed - reconnecting...";
    }
    if (err == TransportError::R2C_SOCKET_ERROR) {
        PLOGE << "ROSBridge socket error - reconnecting...";
    }
}

/******************************************************************************************
 * ROS PUBLISHING
 *****************************************************************************************/

rapidjson::Document create_joint_state_message(const std::array<JointState, armJointCount>& states) {
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
        "j1_scapular_elevation", "j2_scapular_protraction", "j3_shoulder_abduction",
        "j4_shoulder_rotation", "j5_shoulder_flexion", "j6_elbow_flexion", "j7_wrist_pronation"
    };
    for (const char* name : joint_names) {
        names.PushBack(rapidjson::Value(name, allocator), allocator);
    }
    msg.AddMember("name", names, allocator);

    // Positions
    rapidjson::Value positions(rapidjson::kArrayType);
    for (const auto& state : states) {
        positions.PushBack(state.position_rad, allocator);
    }
    msg.AddMember("position", positions, allocator);

    // Velocities (not available, set to zero)
    rapidjson::Value velocities(rapidjson::kArrayType);
    for (size_t i = 0; i < armJointCount; ++i) {
        // TODO: Add velocity data
        velocities.PushBack(0.0, allocator);
    }
    msg.AddMember("velocity", velocities, allocator);

    // Efforts (torques)
    rapidjson::Value efforts(rapidjson::kArrayType);
    for (const auto& state : states) {
        efforts.PushBack(state.torque_Nm, allocator);
    }
    msg.AddMember("effort", efforts, allocator);

    return msg;
}

rapidjson::Document create_transform_message(const Pose& pose, const std::string& parent_frame, const std::string& child_frame) {
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
    header.AddMember("frame_id", rapidjson::Value(parent_frame.c_str(), allocator), allocator);
    msg.AddMember("header", header, allocator);
    msg.AddMember("child_frame_id", rapidjson::Value(child_frame.c_str(), allocator), allocator);

    // Transform: translation (convert mm to m) and rotation
    rapidjson::Value transform(rapidjson::kObjectType);
    
    // Translation
    rapidjson::Value translation(rapidjson::kObjectType);
    translation.AddMember("x", pose.position_mm.x / 1000.0, allocator);
    translation.AddMember("y", pose.position_mm.y / 1000.0, allocator);
    translation.AddMember("z", pose.position_mm.z / 1000.0, allocator);
    transform.AddMember("translation", translation, allocator);
    
    // Rotation (quaternion)
    rapidjson::Value rotation(rapidjson::kObjectType);
    rotation.AddMember("x", pose.orientation.x, allocator);
    rotation.AddMember("y", pose.orientation.y, allocator);
    rotation.AddMember("z", pose.orientation.z, allocator);
    rotation.AddMember("w", pose.orientation.w, allocator);
    transform.AddMember("rotation", rotation, allocator);
    
    msg.AddMember("transform", transform, allocator);

    return msg;
}

rapidjson::Document create_static_transform(const std::string& parent_frame, const std::string& child_frame, double x, double y, double z, double qx, double qy, double qz, double qw) {
    rapidjson::Document msg;
    msg.SetObject();
    auto& allocator = msg.GetAllocator();

    // Header with timestamp (zero for static transforms)
    rapidjson::Value header(rapidjson::kObjectType);
    rapidjson::Value stamp(rapidjson::kObjectType);
    stamp.AddMember("sec", static_cast<int64_t>(0), allocator);
    stamp.AddMember("nanosec", static_cast<uint32_t>(0), allocator);
    header.AddMember("stamp", stamp, allocator);
    header.AddMember("frame_id", rapidjson::Value(parent_frame.c_str(), allocator), allocator);
    msg.AddMember("header", header, allocator);
    msg.AddMember("child_frame_id", rapidjson::Value(child_frame.c_str(), allocator), allocator);

    // Transform: translation and rotation
    rapidjson::Value transform(rapidjson::kObjectType);
    
    // Translation
    rapidjson::Value translation(rapidjson::kObjectType);
    translation.AddMember("x", x, allocator);
    translation.AddMember("y", y, allocator);
    translation.AddMember("z", z, allocator);
    transform.AddMember("translation", translation, allocator);
    
    // Rotation (quaternion)
    rapidjson::Value rotation(rapidjson::kObjectType);
    rotation.AddMember("x", qx, allocator);
    rotation.AddMember("y", qy, allocator);
    rotation.AddMember("z", qz, allocator);
    rotation.AddMember("w", qw, allocator);
    transform.AddMember("rotation", rotation, allocator);
    
    msg.AddMember("transform", transform, allocator);

    return msg;
}

void publish_static_base_frame() {
    if (!g_tf_broadcaster) {
        return;
    }

    // Create static transform from map to base_link
    // map is the root frame (standard ROS convention, matches RViz2 default)
    rapidjson::Document static_transform = create_static_transform("map", "base_link", 
        BASE_LINK_POSITION_X_M, BASE_LINK_POSITION_Y_M, BASE_LINK_POSITION_Z_M, 
        BASE_LINK_ORIENTATION_QX, BASE_LINK_ORIENTATION_QY, BASE_LINK_ORIENTATION_QZ, BASE_LINK_ORIENTATION_QW);
    
    rapidjson::Document transforms_array;
    transforms_array.SetArray();
    auto& allocator = transforms_array.GetAllocator();
    
    rapidjson::Value tf_value;
    tf_value.CopyFrom(static_transform, allocator);
    transforms_array.PushBack(tf_value, allocator);
    
    // Publish static transform (only needs to be done once)
    g_tf_broadcaster->SendStaticTransforms(transforms_array);
}

void publish_tf_transforms() {
    if (!g_research_interface || !g_tf_broadcaster) {
        return;
    }

    auto poses = g_research_interface->poses();
    
    // Create transform array for both end effectors
    rapidjson::Document transforms_array;
    transforms_array.SetArray();
    auto& allocator = transforms_array.GetAllocator();

    // Left end effector transform
    rapidjson::Document left_tf = create_transform_message(poses.leftEndEffector, "base_link", "left_end_effector");
    rapidjson::Value left_tf_value;
    left_tf_value.CopyFrom(left_tf, allocator);
    transforms_array.PushBack(left_tf_value, allocator);

    // Right end effector transform
    rapidjson::Document right_tf = create_transform_message(poses.rightEndEffector, "base_link", "right_end_effector");
    rapidjson::Value right_tf_value;
    right_tf_value.CopyFrom(right_tf, allocator);
    transforms_array.PushBack(right_tf_value, allocator);

    // Publish both transforms
    g_tf_broadcaster->SendTransforms(transforms_array);
}

void publish_joint_states() {
    if (!g_research_interface || !g_left_joint_state_pub || !g_right_joint_state_pub) {
        return;
    }

    auto joint_states = g_research_interface->joints();
    auto left_states = joint_states.leftArm.getOrderedStates();
    auto right_states = joint_states.rightArm.getOrderedStates();

    // Create and publish left arm joint states
    rapidjson::Document left_msg = create_joint_state_message(left_states);
    g_left_joint_state_pub->Publish(left_msg);

    // Create and publish right arm joint states
    rapidjson::Document right_msg = create_joint_state_message(right_states);
    g_right_joint_state_pub->Publish(right_msg);
}

/******************************************************************************************
 * MAIN
 *****************************************************************************************/

void print_help(const char* program_name) {
    PLOGI << "Harmony ROS Interface - ROS interface for Harmony research interface using rosbridge";
    PLOGI << "Usage: " << program_name << " [LOOP_FREQUENCY_HZ]";
    PLOGI << "  LOOP_FREQUENCY_HZ    Publishing frequency in Hz (default: " << DEFAULT_LOOP_FREQUENCY_HZ << ")";
    PLOGI << "  -h, --help           Show this help message";
    PLOGI << "Environment: ROSBRIDGE_HOST (default: 127.0.0.1), ROSBRIDGE_PORT (default: 9090)";
    PLOGI << "Example: " << program_name << " 200  # Loop at 200 Hz";
}

void print_connection_error(const int ros_port) {
    PLOGE << "Failed to connect to ROSBridge server!";
    PLOGE << "Please ensure that:";
    PLOGE << "1. ROS/ROS2 is running";
    PLOGE << "2. ROSBridge server is running:";
    PLOGE << "   For ROS2: ros2 run rosbridge_server rosbridge_websocket";
    PLOGE << "   For ROS1: roslaunch rosbridge_server rosbridge_websocket.launch";
    PLOGE << "3. The server is listening on port " << ros_port;
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
        print_connection_error(ros_port);
        return 1;
    }
    PLOGI << "Successfully connected to ROSBridge server!";

    // Create ROS topics for left and right arms
    ROSTopic left_joint_state_pub(ros_bridge, "/harmony/left/joint_states", "sensor_msgs/JointState");
    ROSTopic right_joint_state_pub(ros_bridge, "/harmony/right/joint_states", "sensor_msgs/JointState");
    g_left_joint_state_pub = &left_joint_state_pub;
    g_right_joint_state_pub = &right_joint_state_pub;
    PLOGI << "Created joint state publishers:";
    PLOGI << "  Left arm:  /harmony/left/joint_states";
    PLOGI << "  Right arm: /harmony/right/joint_states";

    // Create TF broadcaster for end effector poses
    ROSTFBroadcaster tf_broadcaster(ros_bridge);
    g_tf_broadcaster = &tf_broadcaster;
    PLOGI << "Created TF broadcaster for end effector poses";
    
    // Publish static transform from map to base_link (needed for RViz2)
    publish_static_base_frame();
    PLOGI << "Published static transform: map -> base_link";

    // Main loop setup
    auto loop_period = std::chrono::microseconds(static_cast<int>(1000000.0 / loop_frequency_hz));
    auto next_cycle = std::chrono::steady_clock::now();
    int loop_count = -1;
    auto start_time = std::chrono::steady_clock::now();
    int publishes_per_log = static_cast<int>(loop_frequency_hz / LOG_FREQUENCY_HZ);

    // Main publishing loop
    while (true) { // TODO: Add a condition to exit the loop
        publish_joint_states();
        publish_tf_transforms();
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
                
                auto last_left_msg = g_left_joint_state_pub->GetLastPublishedMessage();
                auto last_right_msg = g_right_joint_state_pub->GetLastPublishedMessage();

                if (!last_left_msg.empty()) {
                    PLOGI << "Last published left arm message: " << last_left_msg;
                }
                if (!last_right_msg.empty()) {
                    PLOGI << "Last published right arm message: " << last_right_msg;
                }
            }
        }

        next_cycle += loop_period;
        std::this_thread::sleep_until(next_cycle);
    }

    return 0;
}

