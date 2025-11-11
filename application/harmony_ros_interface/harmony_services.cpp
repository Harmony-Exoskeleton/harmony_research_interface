/**
 * @file harmony_services.cpp
 * @brief ROS service implementations for Harmony research interface
 */

#include "harmony_services.h"
#include "message_utils.h"
#include "plog/Log.h"
#include "joint_states.h"
#include "sizes.h"
#include "shared_memory.h"
#include "shared_memory_manager.h"
#include "arm_controller.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_service_response_msg.h"
#include "types.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <array>
#include <string>
#include <cstring>

using namespace rosbridge2cpp;
using namespace harmony;

// Global service instances
ROSService* g_get_state_service = nullptr;
ROSService* g_get_state_left_service = nullptr;
ROSService* g_get_state_right_service = nullptr;
ROSService* g_enable_left_service = nullptr;
ROSService* g_enable_right_service = nullptr;
ROSService* g_enable_impedance_mode_left_service = nullptr;
ROSService* g_enable_impedance_mode_right_service = nullptr;
ROSService* g_enable_torque_mode_left_service = nullptr;
ROSService* g_enable_torque_mode_right_service = nullptr;
ROSService* g_enable_gravity_left_service = nullptr;
ROSService* g_enable_gravity_right_service = nullptr;
ROSService* g_enable_shr_left_service = nullptr;
ROSService* g_enable_shr_right_service = nullptr;
ROSService* g_enable_constraints_left_service = nullptr;
ROSService* g_enable_constraints_right_service = nullptr;
ROSService* g_reset_shared_memory_service = nullptr;

// Global enable state variables (start with both disabled)
bool g_left_arm_enabled = false;
bool g_right_arm_enabled = false;

// Global control mode tracking (start with harmony mode)
ControlMode g_left_arm_mode = ControlMode::harmony;
ControlMode g_right_arm_mode = ControlMode::harmony;

// Helper function to convert ControlMode to string
static std::string modeToString(ControlMode mode) {
    switch (mode) {
        case ControlMode::harmony: return "harmony";
        case ControlMode::impedance: return "impedance";
        case ControlMode::torque: return "torque";
        default: return "harmony";
    }
}

// Helper function to get the Harmony state
rapidjson::Document getHarmonyState(ResearchInterface* research_interface, bool return_left, bool return_right) {
    if (!research_interface) {
        rapidjson::Document empty;
        empty.SetObject();
        return empty;
    }

    rapidjson::Document response;
    response.SetObject();
    auto& allocator = response.GetAllocator();

    // If neither arm is requested, default to both
    if (!return_left && !return_right) {
        return_left = true;
        return_right = true;
        PLOGW << "No arms specified, defaulting to both";
    }

    // Get joint states
    auto joint_states = research_interface->joints();
    auto left_states = joint_states.leftArm.getOrderedStates();
    auto right_states = joint_states.rightArm.getOrderedStates();

    // Get sizes
    auto sizes = research_interface->sizes();
    auto left_sizes = sizes.leftArm.getOrderedSizes_mm();
    auto right_sizes = sizes.rightArm.getOrderedSizes_mm();

    // Add left arm data if requested
    if (return_left) {
        // Create joint state message
    rapidjson::Document left_joint_state = create_joint_state_message(left_states);
    rapidjson::Value left_joint_state_value;
    left_joint_state_value.CopyFrom(left_joint_state, allocator);
    response.AddMember("left_joint_state", left_joint_state_value, allocator);

    // Add left sizes array
    rapidjson::Value left_sizes_array(rapidjson::kArrayType);
    for (const auto& size : left_sizes) {
        left_sizes_array.PushBack(size, allocator);
    }
    response.AddMember("left_sizes", left_sizes_array, allocator);
    
    // Add left enable state
    response.AddMember("left_enabled", g_left_arm_enabled, allocator);
    
    // Add left mode
    response.AddMember("left_mode", rapidjson::Value(modeToString(g_left_arm_mode).c_str(), allocator), allocator);
    }

    // Add right arm data if requested
    if (return_right) {
        // Create joint state message
        rapidjson::Document right_joint_state = create_joint_state_message(right_states);
        rapidjson::Value right_joint_state_value;
        right_joint_state_value.CopyFrom(right_joint_state, allocator);
        response.AddMember("right_joint_state", right_joint_state_value, allocator);

    // Add right sizes array
    rapidjson::Value right_sizes_array(rapidjson::kArrayType);
    for (const auto& size : right_sizes) {
        right_sizes_array.PushBack(size, allocator);
    }
    response.AddMember("right_sizes", right_sizes_array, allocator);
    
    // Add right enable state
    response.AddMember("right_enabled", g_right_arm_enabled, allocator);
    
    // Add right mode
    response.AddMember("right_mode", rapidjson::Value(modeToString(g_right_arm_mode).c_str(), allocator), allocator);
    }

    PLOGI << "Get Harmony state for " << (return_left ? "left" : "") << (return_left && return_right ? " and " : "") << (return_right ? "right" : "");

    return response;
}

// Helper function to enable/disable harmony mode for an arm
static bool enableHarmony(ResearchInterface* research_interface, bool is_left, bool enable) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    if (enable) {
        // Activate harmony mode by removing override
        controller->removeOverride();
        ControlMode& current_mode = is_left ? g_left_arm_mode : g_right_arm_mode;
        current_mode = ControlMode::harmony;
        
        // Verify mode is set to harmony
        if (controller->getMode() != harmony::ArmController::Mode::harmony) {
            PLOGE << (is_left ? "Left" : "Right") << " arm failed to set mode to harmony";
            return false;
        }
        
        PLOGI << (is_left ? "Left" : "Right") << " arm set to harmony mode";
    } else {
        // When disabling, just remove override (arm will be disabled but mode stays)
        controller->removeOverride();
        PLOGI << (is_left ? "Left" : "Right") << " arm override removed";
    }
    
    return true;
}

// Helper function to set control mode for an arm
static bool setHarmonyControlMode(ResearchInterface* research_interface, bool is_left, ControlMode mode) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    // Stop current mode if different - remove existing overrides
    ControlMode& current_mode = is_left ? g_left_arm_mode : g_right_arm_mode;
    if (current_mode != mode) {
        controller->removeOverride();
    }

    // Set new mode
    switch (mode) {
        case ControlMode::harmony:
            // Activate harmony mode by removing override
            controller->removeOverride();
            current_mode = ControlMode::harmony;
            
            // Verify mode is set to harmony
            if (controller->getMode() != harmony::ArmController::Mode::harmony) {
                PLOGE << (is_left ? "Left" : "Right") << " arm failed to set mode to harmony";
                return false;
            }
            PLOGI << (is_left ? "Left" : "Right") << " arm set to harmony mode";
            break;

        case ControlMode::impedance: {
            // Get current joint positions to prevent arm from jumping
            auto joint_states = research_interface->joints();
            auto states = is_left ? joint_states.leftArm.getOrderedStates() : joint_states.rightArm.getOrderedStates();
            
            // Create overrides with current positions, zero stiffness/torque (will be set via topics)
            std::array<harmony::JointOverride, harmony::armJointCount> overrides;
            for (size_t i = 0; i < harmony::armJointCount; i++) {
                overrides[i] = {
                    states[i].position_rad,  // desired position (current, to prevent jump)
                    0.0,                      // desired stiffness (will be set via topics)
                    0.0                       // desired torque (zero for impedance)
                };
            }
            
            // Activate jointsOverride mode by setting override
            harmony::ArmJointsOverride override(overrides);
            controller->setJointsOverride(override);
            current_mode = ControlMode::impedance;
            
            // Set enabled flag to true when activating override mode
            if (is_left) {
                g_left_arm_enabled = true;
            } else {
                g_right_arm_enabled = true;
            }
            
            // Verify mode is set to jointsOverride
            if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
                PLOGE << (is_left ? "Left" : "Right") << " arm failed to set mode to jointsOverride for impedance";
                return false;
            }
            PLOGI << (is_left ? "Left" : "Right") << " arm set to impedance mode (control values via topics)";
            break;
        }

        case ControlMode::torque: {
            // Create overrides with zero values (will be set via topics)
            std::array<harmony::JointOverride, harmony::armJointCount> overrides;
            for (size_t i = 0; i < harmony::armJointCount; i++) {
                overrides[i] = {
                    0.0,  // desired position (zero)
                    0.0,  // desired stiffness (zero)
                    0.0   // desired torque (will be set via topics)
                };
            }
            
            // Activate jointsOverride mode by setting override
            harmony::ArmJointsOverride override(overrides);
            controller->setJointsOverride(override);
            current_mode = ControlMode::torque;
            
            // Set enabled flag to true when activating override mode
            if (is_left) {
                g_left_arm_enabled = true;
            } else {
                g_right_arm_enabled = true;
            }
            
            // Verify mode is set to jointsOverride
            if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
                PLOGE << (is_left ? "Left" : "Right") << " arm failed to set mode to jointsOverride for torque";
                return false;
            }
            PLOGI << (is_left ? "Left" : "Right") << " arm set to torque mode (control values via topics)";
            break;
        }
    }

    return true;
}

// Helper function to update gravity compensation flag for an arm
static bool setArmGravity(ResearchInterface* research_interface, bool is_left, bool enable) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    // Check if we're in jointsOverride mode (impedance or torque)
    if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
        PLOGE << (is_left ? "Left" : "Right") << " arm must be in impedance or torque mode to set gravity compensation";
        return false;
    }

    // Get current override state
    auto current_override = controller->getArmJointsOverride();
    if (!current_override) {
        PLOGE << "Failed to get current override state - controller is not in jointsOverride mode";
        return false;
    }

    // Get current overrides and flags
    auto overrides = current_override->getOrderedOverrides();
    bool constraints_enabled = current_override->areShoulderConstraintsEnabled();
    bool gravity_disabled = current_override->isGravityTorqueDisabled();
    bool shr_disabled = current_override->isSHRTorqueDisabled();

    // Update gravity flag: enable means gravityTorqueDisabled = false
    bool new_gravity_disabled = !enable;
    harmony::ArmJointsOverride new_override(overrides, constraints_enabled, new_gravity_disabled, shr_disabled);
    controller->setJointsOverride(new_override);

    PLOGI << (is_left ? "Left" : "Right") << " arm gravity compensation " << (enable ? "enabled" : "disabled");
    return true;
}

// Helper function to update SHR flag for an arm
static bool setArmSHR(ResearchInterface* research_interface, bool is_left, bool enable) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    // Check if we're in jointsOverride mode (impedance or torque)
    if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
        PLOGE << (is_left ? "Left" : "Right") << " arm must be in impedance or torque mode to set SHR";
        return false;
    }

    // Get current override state
    auto current_override = controller->getArmJointsOverride();
    if (!current_override) {
        PLOGE << "Failed to get current override state - controller is not in jointsOverride mode";
        return false;
    }

    // Get current overrides and flags
    auto overrides = current_override->getOrderedOverrides();
    bool constraints_enabled = current_override->areShoulderConstraintsEnabled();
    bool gravity_disabled = current_override->isGravityTorqueDisabled();
    bool shr_disabled = current_override->isSHRTorqueDisabled();

    // Update SHR flag: enable means SHRTorqueDisabled = false
    bool new_shr_disabled = !enable;
    harmony::ArmJointsOverride new_override(overrides, constraints_enabled, gravity_disabled, new_shr_disabled);
    controller->setJointsOverride(new_override);

    PLOGI << (is_left ? "Left" : "Right") << " arm SHR " << (enable ? "enabled" : "disabled");
    return true;
}

// Helper function to update constraints flag for an arm
static bool setArmConstraints(ResearchInterface* research_interface, bool is_left, bool enable) {
    if (!research_interface) {
        PLOGE << "Research interface is null";
        return false;
    }

    auto controller = is_left ? research_interface->makeLeftArmController() : research_interface->makeRightArmController();
    if (!controller->init()) {
        PLOGE << "Failed to initialize " << (is_left ? "left" : "right") << " arm controller";
        return false;
    }

    // Check if we're in jointsOverride mode (impedance or torque)
    if (controller->getMode() != harmony::ArmController::Mode::jointsOverride) {
        PLOGE << (is_left ? "Left" : "Right") << " arm must be in impedance or torque mode to set constraints";
        return false;
    }

    // Get current override state
    auto current_override = controller->getArmJointsOverride();
    if (!current_override) {
        PLOGE << "Failed to get current override state - controller is not in jointsOverride mode";
        return false;
    }

    // Get current overrides and flags
    auto overrides = current_override->getOrderedOverrides();
    bool constraints_enabled = current_override->areShoulderConstraintsEnabled();
    bool gravity_disabled = current_override->isGravityTorqueDisabled();
    bool shr_disabled = current_override->isSHRTorqueDisabled();

    // Update constraints flag
    harmony::ArmJointsOverride new_override(overrides, enable, gravity_disabled, shr_disabled);
    controller->setJointsOverride(new_override);

    PLOGI << (is_left ? "Left" : "Right") << " arm constraints " << (enable ? "enabled" : "disabled");
    return true;
}

// Helper function to create the service callback
auto create_get_state_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/get_state";
        
        // Create response with both arms' joint states and sizes
        rapidjson::Document response_data = getHarmonyState(research_interface, true, true);
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // For std_srvs/Trigger response structure:
        // - success: bool
        // - message: string
        // We'll put our JSON data in the message field as a string, and also include it as raw data
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        response_data.Accept(writer);
        std::string json_string = buffer.GetString();
        
        // Create response values matching std_srvs/Trigger structure
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", true, allocator);
        response_values.AddMember("message", rapidjson::Value(json_string.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/get_state";
    };
}

bool setup_get_state_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    // Using std_srvs/Trigger which has a string message field we can use for JSON data
    static ROSService get_state_service(ros_bridge, "/harmony/get_state", "std_srvs/Trigger");
    g_get_state_service = &get_state_service;
    
    // Create service callback using helper function
    auto service_callback = create_get_state_callback(ros_bridge, research_interface);

    // Advertise the service (will re-advertise if already advertised)
    bool success = get_state_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/get_state";
    } else {
        PLOGE << "Failed to advertise service: /harmony/get_state";
    }
    return success;
}

// Helper function to create the left arm get_state service callback
auto create_get_state_left_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/left/get_state";
        
        // Create response with left arm joint states and sizes
        rapidjson::Document response_data = getHarmonyState(research_interface, true, false);
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // For std_srvs/Trigger response structure:
        // - success: bool
        // - message: string
        // We'll put our JSON data in the message field as a string
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        response_data.Accept(writer);
        std::string json_string = buffer.GetString();
        
        // Create response values matching std_srvs/Trigger structure
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", true, allocator);
        response_values.AddMember("message", rapidjson::Value(json_string.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/left/get_state";
    };
}

// Helper function to create the right arm get_state service callback
auto create_get_state_right_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/right/get_state";
        
        // Create response with right arm joint states and sizes
        rapidjson::Document response_data = getHarmonyState(research_interface, false, true);
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // For std_srvs/Trigger response structure:
        // - success: bool
        // - message: string
        // We'll put our JSON data in the message field as a string
        rapidjson::StringBuffer buffer;
        rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
        response_data.Accept(writer);
        std::string json_string = buffer.GetString();
        
        // Create response values matching std_srvs/Trigger structure
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", true, allocator);
        response_values.AddMember("message", rapidjson::Value(json_string.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/right/get_state";
    };
}

bool setup_get_state_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService get_state_left_service(ros_bridge, "/harmony/left/get_state", "std_srvs/Trigger");
    g_get_state_left_service = &get_state_left_service;
    
    // Create service callback
    auto service_callback = create_get_state_left_callback(ros_bridge, research_interface);

    // Advertise the service (will re-advertise if already advertised)
    bool success = get_state_left_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/get_state";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/get_state";
    }
    return success;
}

bool setup_get_state_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService get_state_right_service(ros_bridge, "/harmony/right/get_state", "std_srvs/Trigger");
    g_get_state_right_service = &get_state_right_service;
    
    // Create service callback
    auto service_callback = create_get_state_right_callback(ros_bridge, research_interface);

    // Advertise the service (will re-advertise if already advertised)
    bool success = get_state_right_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/get_state";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/get_state";
    }
    return success;
}

// Helper function to create the left arm disable_override_mode service callback
auto create_disable_override_mode_left_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/left/disable_override_mode";
        
        // Disable arm and set to harmony mode
        g_left_arm_enabled = false;
        bool success = setHarmonyControlMode(research_interface, true, ControlMode::harmony);
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        // For std_srvs/Trigger response structure:
        // - success: bool
        // - message: string
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? "Left arm disabled and set to harmony mode" : "Failed to disable left arm";
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/left/disable_override_mode";
    };
}

// Helper function to create the right arm disable_override_mode service callback
auto create_disable_override_mode_right_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/right/disable_override_mode";
        
        // Disable arm and set to harmony mode
        g_right_arm_enabled = false;
        bool success = setHarmonyControlMode(research_interface, false, ControlMode::harmony);
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        // For std_srvs/Trigger response structure:
        // - success: bool
        // - message: string
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? "Right arm disabled and set to harmony mode" : "Failed to disable right arm";
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/right/disable_override_mode";
    };
}

bool setup_disable_override_mode_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService disable_override_mode_left_service(ros_bridge, "/harmony/left/disable_override_mode", "std_srvs/Trigger");
    g_enable_left_service = &disable_override_mode_left_service;
    
    // Create service callback
    auto service_callback = create_disable_override_mode_left_callback(ros_bridge, research_interface);

    // Advertise the service (will re-advertise if already advertised)
    bool success = disable_override_mode_left_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/disable_override_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/disable_override_mode";
    }
    return success;
}

bool setup_disable_override_mode_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService disable_override_mode_right_service(ros_bridge, "/harmony/right/disable_override_mode", "std_srvs/Trigger");
    g_enable_right_service = &disable_override_mode_right_service;
    
    // Create service callback
    auto service_callback = create_disable_override_mode_right_callback(ros_bridge, research_interface);

    // Advertise the service (will re-advertise if already advertised)
    bool success = disable_override_mode_right_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/disable_override_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/disable_override_mode";
    }
    return success;
}




// Helper function to create impedance mode callback
auto create_enable_impedance_mode_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGD << "Received service request for /harmony/" << arm_name << "/enable_impedance_mode";
        
        bool success = setHarmonyControlMode(research_interface, is_left, ControlMode::impedance);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? (arm_name + " arm set to impedance mode") : ("Failed to set " + arm_name + " arm to impedance mode");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/" << arm_name << "/enable_impedance_mode";
    };
}

// Helper function to create torque mode callback
auto create_enable_torque_mode_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGD << "Received service request for /harmony/" << arm_name << "/enable_torque_mode";
        
        bool success = setHarmonyControlMode(research_interface, is_left, ControlMode::torque);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? (arm_name + " arm set to torque mode") : ("Failed to set " + arm_name + " arm to torque mode");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/" << arm_name << "/enable_torque_mode";
    };
}


bool setup_enable_impedance_mode_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_impedance_mode", "std_srvs/Trigger");
    g_enable_impedance_mode_left_service = &service;
    
    auto callback = create_enable_impedance_mode_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_impedance_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_impedance_mode";
    }
    return success;
}

bool setup_enable_impedance_mode_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_impedance_mode", "std_srvs/Trigger");
    g_enable_impedance_mode_right_service = &service;
    
    auto callback = create_enable_impedance_mode_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_impedance_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_impedance_mode";
    }
    return success;
}

bool setup_enable_torque_mode_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_torque_mode", "std_srvs/Trigger");
    g_enable_torque_mode_left_service = &service;
    
    auto callback = create_enable_torque_mode_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_torque_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_torque_mode";
    }
    return success;
}

bool setup_enable_torque_mode_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_torque_mode", "std_srvs/Trigger");
    g_enable_torque_mode_right_service = &service;
    
    auto callback = create_enable_torque_mode_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_torque_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_torque_mode";
    }
    return success;
}


// Helper function to create enable_gravity callback
auto create_enable_gravity_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGD << "Received service request for /harmony/" << arm_name << "/enable_gravity";
        
        // Parse request to get enable/disable flag
        // std_srvs/SetBool has a "data" field (boolean) in the request
        bool enable = true;
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("data") && request.args_json_["data"].IsBool()) {
                enable = request.args_json_["data"].GetBool();
            }
        }
        
        bool success = setArmGravity(research_interface, is_left, enable);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? 
            (arm_name + " arm gravity compensation " + (enable ? "enabled" : "disabled")) : 
            ("Failed to " + std::string(enable ? "enable" : "disable") + " gravity compensation for " + arm_name + " arm");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/" << arm_name << "/enable_gravity";
    };
}

// Helper function to create enable_shr callback
auto create_enable_shr_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGD << "Received service request for /harmony/" << arm_name << "/enable_shr";
        
        // Parse request to get enable/disable flag
        // std_srvs/SetBool has a "data" field (boolean) in the request
        bool enable = true;
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("data") && request.args_json_["data"].IsBool()) {
                enable = request.args_json_["data"].GetBool();
            }
        }
        
        bool success = setArmSHR(research_interface, is_left, enable);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? 
            (arm_name + " arm SHR " + (enable ? "enabled" : "disabled")) : 
            ("Failed to " + std::string(enable ? "enable" : "disable") + " SHR for " + arm_name + " arm");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/" << arm_name << "/enable_shr";
    };
}

// Helper function to create enable_constraints callback
auto create_enable_constraints_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGD << "Received service request for /harmony/" << arm_name << "/enable_constraints";
        
        // Parse request to get enable/disable flag
        // std_srvs/SetBool has a "data" field (boolean) in the request
        bool enable = true;
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("data") && request.args_json_["data"].IsBool()) {
                enable = request.args_json_["data"].GetBool();
            }
        }
        
        bool success = setArmConstraints(research_interface, is_left, enable);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? 
            (arm_name + " arm constraints " + (enable ? "enabled" : "disabled")) : 
            ("Failed to " + std::string(enable ? "enable" : "disable") + " constraints for " + arm_name + " arm");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/" << arm_name << "/enable_constraints";
    };
}

bool setup_enable_gravity_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_gravity", "std_srvs/SetBool");
    g_enable_gravity_left_service = &service;
    
    auto callback = create_enable_gravity_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_gravity";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_gravity";
    }
    return success;
}

bool setup_enable_gravity_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_gravity", "std_srvs/SetBool");
    g_enable_gravity_right_service = &service;
    
    auto callback = create_enable_gravity_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_gravity";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_gravity";
    }
    return success;
}

bool setup_enable_shr_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_shr", "std_srvs/SetBool");
    g_enable_shr_left_service = &service;
    
    auto callback = create_enable_shr_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_shr";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_shr";
    }
    return success;
}

bool setup_enable_shr_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_shr", "std_srvs/SetBool");
    g_enable_shr_right_service = &service;
    
    auto callback = create_enable_shr_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_shr";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_shr";
    }
    return success;
}

bool setup_enable_constraints_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_constraints", "std_srvs/SetBool");
    g_enable_constraints_left_service = &service;
    
    auto callback = create_enable_constraints_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_constraints";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_constraints";
    }
    return success;
}

bool setup_enable_constraints_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_constraints", "std_srvs/SetBool");
    g_enable_constraints_right_service = &service;
    
    auto callback = create_enable_constraints_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_constraints";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_constraints";
    }
    return success;
}

// Helper function to reset shared memory
static bool resetSharedMemory() {
    // Initialize shared memory managers (as non-owners so we don't destroy them)
    harmony::SharedMemoryManager<harmony::DataFromHarmony> harmonyStateManager(harmony::dataFromHarmonyMemId, false);
    harmony::SharedMemoryManager<harmony::DataToHarmony> harmonyCommandManager(harmony::dataToHarmonyMemId, false);
    harmony::SharedMemoryManager<harmony::ArmControllerState> leftControllerStateManager(
        harmony::leftArmControllerMemId, false);
    harmony::SharedMemoryManager<harmony::ArmControllerState> rightControllerStateManager(
        harmony::rightArmControllerMemId, false);

    // Initialize all managers
    bool success = harmonyStateManager.init() && 
                   harmonyCommandManager.init() && 
                   leftControllerStateManager.init() && 
                   rightControllerStateManager.init();

    if (!success) {
        PLOGE << "Failed to initialize shared memory managers for reset";
        return false;
    }

    // Reset DataFromHarmony - zero out all data
    std::memset(harmonyStateManager.data, 0, sizeof(harmony::DataFromHarmony));

    // Reset DataToHarmony - zero out all data
    std::memset(harmonyCommandManager.data, 0, sizeof(harmony::DataToHarmony));

    // Reset left ArmControllerState to defaults
    harmony::ArmControllerState defaultControllerState;
    defaultControllerState.mode = harmony::ArmController::Mode::harmony;
    defaultControllerState.shoulderConstraintsEnabled = true;
    defaultControllerState.gravityTorqueDisabled = false;
    defaultControllerState.SHRTorqueDisabled = false;
    // Zero out joint overrides
    std::memset(defaultControllerState.jointsOverride, 0, 
               sizeof(harmony::JointOverride) * harmony::armJointCount);
    
    *leftControllerStateManager.data = defaultControllerState;

    // Reset right ArmControllerState to defaults
    *rightControllerStateManager.data = defaultControllerState;

    PLOGI << "Shared memory reset complete";
    return true;
}

// Helper function to create reset_shared_memory callback
auto create_reset_shared_memory_callback(ROSBridge& ros_bridge) {
    return [&ros_bridge](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGD << "Received service request for /harmony/reset_shared_memory";
        
        bool success = resetSharedMemory();
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? 
            "Shared memory reset successfully" : 
            "Failed to reset shared memory";
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGD << "Sent service response for /harmony/reset_shared_memory";
    };
}

bool setup_reset_shared_memory_service(ROSBridge& ros_bridge) {
    static ROSService service(ros_bridge, "/harmony/reset_shared_memory", "std_srvs/Trigger");
    g_reset_shared_memory_service = &service;
    
    auto callback = create_reset_shared_memory_callback(ros_bridge);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/reset_shared_memory";
    } else {
        PLOGE << "Failed to advertise service: /harmony/reset_shared_memory";
    }
    return success;
}
