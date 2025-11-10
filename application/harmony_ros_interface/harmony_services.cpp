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

// Global enable state variables (start with both disabled)
bool g_left_arm_enabled = false;
bool g_right_arm_enabled = false;

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
