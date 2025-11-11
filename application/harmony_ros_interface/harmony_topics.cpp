/**
 * @file harmony_topics.cpp
 * @brief ROS topic subscriber implementations for Harmony research interface
 */

#include "harmony_topics.h"
#include "harmony_services.h"
#include "plog/Log.h"
#include "messages/rosbridge_publish_msg.h"
#include <array>
#include <string>

using namespace rosbridge2cpp;
using namespace harmony;

// Global subscriber instances
ROSTopic* g_left_desired_torque_sub = nullptr;
ROSTopic* g_right_desired_torque_sub = nullptr;
ROSTopic* g_left_desired_stiffness_sub = nullptr;
ROSTopic* g_right_desired_stiffness_sub = nullptr;
ROSTopic* g_left_desired_position_sub = nullptr;
ROSTopic* g_right_desired_position_sub = nullptr;

// Static variable to store research interface for callbacks
static ResearchInterface* s_research_interface = nullptr;

// Helper function to parse Float64MultiArray message
static bool parse_float64_array(const ROSBridgePublishMsg &message, std::array<double, harmony::armJointCount>& values) {
    if (message.msg_json_.IsNull()) {
        PLOGW << "Received message with null JSON";
        return false;
    }

    // Parse Float64MultiArray message
    if (!message.msg_json_.HasMember("data") || !message.msg_json_["data"].IsArray()) {
        PLOGW << "Invalid message format: missing or invalid 'data' field";
        return false;
    }

    auto& data = message.msg_json_["data"];
    if (data.Size() != harmony::armJointCount) {
        PLOGE << "Invalid array size: expected " << harmony::armJointCount << ", got " << data.Size();
        return false;
    }

    // Extract values into array
    for (size_t i = 0; i < harmony::armJointCount; i++) {
        if (!data[i].IsNumber()) {
            PLOGE << "Invalid value at index " << i << ": not a number";
            return false;
        }
        values[i] = data[i].GetDouble();
    }

    return true;
}

// Helper function to update joint override values
// Updates only the specified fields (torque, stiffness, or position) while preserving others
bool updateJointOverride(ResearchInterface* research_interface, bool is_left,
                         const std::array<double, harmony::armJointCount>* torque_values,
                         const std::array<double, harmony::armJointCount>* stiffness_values,
                         const std::array<double, harmony::armJointCount>* position_values,
                         ControlMode required_mode) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    // Check current mode matches required mode
    ControlMode current_mode = is_left ? g_left_arm_mode : g_right_arm_mode;
    if (current_mode != required_mode) {
        PLOGW << (is_left ? "Left" : "Right") << " arm is not in " 
              << (required_mode == ControlMode::torque ? "torque" : "impedance") 
              << " mode, ignoring command";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    // Check if we're in jointsOverride mode
    if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
        PLOGW << (is_left ? "Left" : "Right") << " arm is not in override mode (harmony mode), ignoring command";
        return false;
    }

    // Get current override state
    auto current_override = controller->getArmJointsOverride();
    if (!current_override) {
        PLOGW << "Failed to get current override state - controller is not in jointsOverride mode";
        return false;
    }

    // Get current overrides and flags
    auto overrides = current_override->getOrderedOverrides();
    bool constraints_enabled = current_override->areShoulderConstraintsEnabled();
    bool gravity_disabled = current_override->isGravityTorqueDisabled();
    bool shr_disabled = current_override->isSHRTorqueDisabled();

    // Update only the specified fields
    for (size_t i = 0; i < harmony::armJointCount; i++) {
        if (torque_values) {
            overrides[i].desiredTorque_Nm = (*torque_values)[i];
        }
        if (stiffness_values) {
            overrides[i].desiredStiffness_Nm_per_rad = (*stiffness_values)[i];
        }
        if (position_values) {
            overrides[i].desiredPosition_rad = (*position_values)[i];
        }
    }

    // Create new override with updated values
    harmony::ArmJointsOverride new_override(overrides, constraints_enabled, gravity_disabled, shr_disabled);
    controller->setJointsOverride(new_override);

    std::string update_types;
    if (torque_values) update_types += "torque ";
    if (stiffness_values) update_types += "stiffness ";
    if (position_values) update_types += "position";
    PLOGI << "Updated Joint Overrides for " << (is_left ? "left" : "right") << " arm: " << update_types;

    return true;
}

// Torque command callbacks
void desired_torque_callback_left(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring torque command";
        return;
    }

    std::array<double, harmony::armJointCount> torque_values;
    if (!parse_float64_array(message, torque_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, true, &torque_values, nullptr, nullptr, ControlMode::torque)) {
        PLOGD << "Updated left arm torque values";
    }
}

void desired_torque_callback_right(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring torque command";
        return;
    }

    std::array<double, harmony::armJointCount> torque_values;
    if (!parse_float64_array(message, torque_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, false, &torque_values, nullptr, nullptr, ControlMode::torque)) {
        PLOGD << "Updated right arm torque values";
    }
}

// Stiffness command callbacks
void desired_stiffness_callback_left(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring stiffness command";
        return;
    }

    std::array<double, harmony::armJointCount> stiffness_values;
    if (!parse_float64_array(message, stiffness_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, true, nullptr, &stiffness_values, nullptr, ControlMode::impedance)) {
        PLOGD << "Updated left arm stiffness values";
    }
}

void desired_stiffness_callback_right(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring stiffness command";
        return;
    }

    std::array<double, harmony::armJointCount> stiffness_values;
    if (!parse_float64_array(message, stiffness_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, false, nullptr, &stiffness_values, nullptr, ControlMode::impedance)) {
        PLOGD << "Updated right arm stiffness values";
    }
}

// Position command callbacks
void desired_position_callback_left(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring position command";
        return;
    }

    std::array<double, harmony::armJointCount> position_values;
    if (!parse_float64_array(message, position_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, true, nullptr, nullptr, &position_values, ControlMode::impedance)) {
        PLOGD << "Updated left arm position values";
    }
}

void desired_position_callback_right(const ROSBridgePublishMsg &message) {
    if (!s_research_interface) {
        PLOGW << "Research interface not initialized, ignoring position command";
        return;
    }

    std::array<double, harmony::armJointCount> position_values;
    if (!parse_float64_array(message, position_values)) {
        return;
    }

    if (updateJointOverride(s_research_interface, false, nullptr, nullptr, &position_values, ControlMode::impedance)) {
        PLOGD << "Updated right arm position values";
    }
}

bool setup_joint_command_subscribers(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    if (!research_interface) {
        PLOGE << "Research interface is null, cannot setup subscribers";
        return false;
    }

    // Store research interface for callbacks
    s_research_interface = research_interface;

    // Create and subscribe to left arm torque topic
    static ROSTopic left_torque_sub(ros_bridge, "/harmony/left/desired_torque", "std_msgs/Float64MultiArray");
    g_left_desired_torque_sub = &left_torque_sub;
    auto left_torque_handle = left_torque_sub.Subscribe(desired_torque_callback_left);
    if (left_torque_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/left/desired_torque";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/left/desired_torque";
    }

    // Create and subscribe to right arm torque topic
    static ROSTopic right_torque_sub(ros_bridge, "/harmony/right/desired_torque", "std_msgs/Float64MultiArray");
    g_right_desired_torque_sub = &right_torque_sub;
    auto right_torque_handle = right_torque_sub.Subscribe(desired_torque_callback_right);
    if (right_torque_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/right/desired_torque";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/right/desired_torque";
    }

    // Create and subscribe to left arm stiffness topic
    static ROSTopic left_stiffness_sub(ros_bridge, "/harmony/left/desired_stiffness", "std_msgs/Float64MultiArray");
    g_left_desired_stiffness_sub = &left_stiffness_sub;
    auto left_stiffness_handle = left_stiffness_sub.Subscribe(desired_stiffness_callback_left);
    if (left_stiffness_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/left/desired_stiffness";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/left/desired_stiffness";
    }

    // Create and subscribe to right arm stiffness topic
    static ROSTopic right_stiffness_sub(ros_bridge, "/harmony/right/desired_stiffness", "std_msgs/Float64MultiArray");
    g_right_desired_stiffness_sub = &right_stiffness_sub;
    auto right_stiffness_handle = right_stiffness_sub.Subscribe(desired_stiffness_callback_right);
    if (right_stiffness_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/right/desired_stiffness";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/right/desired_stiffness";
    }

    // Create and subscribe to left arm position topic
    static ROSTopic left_position_sub(ros_bridge, "/harmony/left/desired_position", "std_msgs/Float64MultiArray");
    g_left_desired_position_sub = &left_position_sub;
    auto left_position_handle = left_position_sub.Subscribe(desired_position_callback_left);
    if (left_position_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/left/desired_position";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/left/desired_position";
    }

    // Create and subscribe to right arm position topic
    static ROSTopic right_position_sub(ros_bridge, "/harmony/right/desired_position", "std_msgs/Float64MultiArray");
    g_right_desired_position_sub = &right_position_sub;
    auto right_position_handle = right_position_sub.Subscribe(desired_position_callback_right);
    if (right_position_handle.IsValid()) {
        PLOGI << "Subscribed to topic: /harmony/right/desired_position";
    } else {
        PLOGW << "Failed to subscribe to topic: /harmony/right/desired_position";
    }

    return true;
}

