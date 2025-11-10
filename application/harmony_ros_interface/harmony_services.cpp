/**
 * @file harmony_services.cpp
 * @brief ROS service implementations for Harmony research interface
 */

#include "harmony_services.h"
#include "message_utils.h"
#include "plog/Log.h"
#include "joint_states.h"
#include "sizes.h"
#include "messages/rosbridge_call_service_msg.h"
#include "messages/rosbridge_service_response_msg.h"
#include "types.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <array>
#include <string>

using namespace rosbridge2cpp;
using namespace harmony;

// Global service instances
ROSService* g_get_state_service = nullptr;
ROSService* g_get_state_left_service = nullptr;
ROSService* g_get_state_right_service = nullptr;
ROSService* g_enable_left_service = nullptr;
ROSService* g_enable_right_service = nullptr;
ROSService* g_enable_harmony_mode_left_service = nullptr;
ROSService* g_enable_harmony_mode_right_service = nullptr;
ROSService* g_enable_impedance_mode_left_service = nullptr;
ROSService* g_enable_impedance_mode_right_service = nullptr;
ROSService* g_enable_torque_mode_left_service = nullptr;
ROSService* g_enable_torque_mode_right_service = nullptr;

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

rapidjson::Document create_get_state_response(ResearchInterface* research_interface, bool return_left, bool return_right) {
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

    return response;
}

// Helper function to create the service callback
auto create_get_state_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGI << "Received service request for /harmony/get_state";
        
        // Create response with both arms' joint states and sizes
        rapidjson::Document response_data = create_get_state_response(research_interface, true, true);
        
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
        PLOGI << "Sent service response for /harmony/get_state";
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
        
        PLOGI << "Received service request for /harmony/left/get_state";
        
        // Create response with left arm joint states and sizes
        rapidjson::Document response_data = create_get_state_response(research_interface, true, false);
        
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
        PLOGI << "Sent service response for /harmony/left/get_state";
    };
}

// Helper function to create the right arm get_state service callback
auto create_get_state_right_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        PLOGI << "Received service request for /harmony/right/get_state";
        
        // Create response with right arm joint states and sizes
        rapidjson::Document response_data = create_get_state_response(research_interface, false, true);
        
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
        PLOGI << "Sent service response for /harmony/right/get_state";
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

// Helper function to create the left arm enable service callback
auto create_enable_left_callback(ROSBridge& ros_bridge) {
    return [&ros_bridge](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        // Extract enable state from request
        // std_srvs/SetBool has a "data" field (boolean) in the request
        bool enable = false;
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("data") && request.args_json_["data"].IsBool()) {
                enable = request.args_json_["data"].GetBool();
            }
        }
        
        // Update global enable state
        g_left_arm_enabled = enable;
        
        PLOGI << "Received service request for /harmony/left/enable: " << (enable ? "enabled" : "disabled");
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // For std_srvs/SetBool response structure:
        // - success: bool
        // - message: string
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", true, allocator);
        std::string message = enable ? "Left arm enabled" : "Left arm disabled";
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGI << "Sent service response for /harmony/left/enable";
    };
}

// Helper function to create the right arm enable service callback
auto create_enable_right_callback(ROSBridge& ros_bridge) {
    return [&ros_bridge](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        // Extract enable state from request
        // std_srvs/SetBool has a "data" field (boolean) in the request
        bool enable = false;
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("data") && request.args_json_["data"].IsBool()) {
                enable = request.args_json_["data"].GetBool();
            }
        }
        
        // Update global enable state
        g_right_arm_enabled = enable;
        
        PLOGI << "Received service request for /harmony/right/enable: " << (enable ? "enabled" : "disabled");
        
        // Create service response message
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = true;
        
        // For std_srvs/SetBool response structure:
        // - success: bool
        // - message: string
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", true, allocator);
        std::string message = enable ? "Right arm enabled" : "Right arm disabled";
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        // Copy response data to values_json_
        response.values_json_.CopyFrom(response_values, allocator);
        
        // Send response
        ros_bridge.SendMessage(response);
        PLOGI << "Sent service response for /harmony/right/enable";
    };
}

bool setup_enable_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService enable_left_service(ros_bridge, "/harmony/left/enable", "std_srvs/SetBool");
    g_enable_left_service = &enable_left_service;
    
    // Create service callback
    auto service_callback = create_enable_left_callback(ros_bridge);

    // Advertise the service (will re-advertise if already advertised)
    bool success = enable_left_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable";
    }
    return success;
}

bool setup_enable_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    // Create service instance (static to persist across calls)
    static ROSService enable_right_service(ros_bridge, "/harmony/right/enable", "std_srvs/SetBool");
    g_enable_right_service = &enable_right_service;
    
    // Create service callback
    auto service_callback = create_enable_right_callback(ros_bridge);

    // Advertise the service (will re-advertise if already advertised)
    bool success = enable_right_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable";
    }
    return success;
}

// Helper function to set control mode for an arm
static bool setArmControlMode(ResearchInterface* research_interface, bool is_left, ControlMode mode) {
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
            controller->removeOverride();
            current_mode = ControlMode::harmony;
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
            
            harmony::ArmJointsOverride override(overrides);
            controller->setJointsOverride(override);
            current_mode = ControlMode::impedance;
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
            
            harmony::ArmJointsOverride override(overrides);
            controller->setJointsOverride(override);
            current_mode = ControlMode::torque;
            PLOGI << (is_left ? "Left" : "Right") << " arm set to torque mode (control values via topics)";
            break;
        }
    }

    return true;
}

// Helper function to create harmony mode callback
auto create_enable_harmony_mode_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGI << "Received service request for /harmony/" << arm_name << "/enable_harmony_mode";
        
        bool success = setArmControlMode(research_interface, is_left, ControlMode::harmony);
        
        ROSBridgeServiceResponseMsg response(true);
        response.id_ = request.id_;
        response.service_ = request.service_;
        response.result_ = success;
        
        rapidjson::Document response_values;
        response_values.SetObject();
        response_values.AddMember("success", success, allocator);
        std::string message = success ? (arm_name + " arm set to harmony mode") : ("Failed to set " + arm_name + " arm to harmony mode");
        response_values.AddMember("message", rapidjson::Value(message.c_str(), allocator), allocator);
        
        response.values_json_.CopyFrom(response_values, allocator);
        ros_bridge.SendMessage(response);
        PLOGI << "Sent service response for /harmony/" << arm_name << "/enable_harmony_mode";
    };
}

// Helper function to create impedance mode callback
auto create_enable_impedance_mode_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGI << "Received service request for /harmony/" << arm_name << "/enable_impedance_mode";
        
        bool success = setArmControlMode(research_interface, is_left, ControlMode::impedance);
        
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
        PLOGI << "Sent service response for /harmony/" << arm_name << "/enable_impedance_mode";
    };
}

// Helper function to create torque mode callback
auto create_enable_torque_mode_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface, bool is_left) {
    return [&ros_bridge, research_interface, is_left](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        std::string arm_name = is_left ? "left" : "right";
        PLOGI << "Received service request for /harmony/" << arm_name << "/enable_torque_mode";
        
        bool success = setArmControlMode(research_interface, is_left, ControlMode::torque);
        
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
        PLOGI << "Sent service response for /harmony/" << arm_name << "/enable_torque_mode";
    };
}

bool setup_enable_harmony_mode_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/left/enable_harmony_mode", "std_srvs/Trigger");
    g_enable_harmony_mode_left_service = &service;
    
    auto callback = create_enable_harmony_mode_callback(ros_bridge, research_interface, true);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/left/enable_harmony_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/left/enable_harmony_mode";
    }
    return success;
}

bool setup_enable_harmony_mode_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    static ROSService service(ros_bridge, "/harmony/right/enable_harmony_mode", "std_srvs/Trigger");
    g_enable_harmony_mode_right_service = &service;
    
    auto callback = create_enable_harmony_mode_callback(ros_bridge, research_interface, false);
    bool success = service.Advertise(callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/right/enable_harmony_mode";
    } else {
        PLOGE << "Failed to advertise service: /harmony/right/enable_harmony_mode";
    }
    return success;
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
