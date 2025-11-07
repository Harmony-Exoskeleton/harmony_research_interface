/**
 * @file message_utils.cpp
 * @brief Utility functions for creating ROS messages from Harmony data
 */

#include "message_utils.h"
#include <chrono>

using namespace harmony;

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

rapidjson::Document create_transform(const Pose& pose, const std::string& parent_frame, const std::string& child_frame) {
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

