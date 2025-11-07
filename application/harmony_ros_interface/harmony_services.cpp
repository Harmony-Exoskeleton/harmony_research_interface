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
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <array>
#include <string>

using namespace rosbridge2cpp;
using namespace harmony;

// Global service instance
ROSService* g_get_state_service = nullptr;

rapidjson::Document create_get_state_response(ResearchInterface* research_interface, const std::string& arm_selection) {
    if (!research_interface) {
        rapidjson::Document empty;
        empty.SetObject();
        return empty;
    }

    rapidjson::Document response;
    response.SetObject();
    auto& allocator = response.GetAllocator();

    // Determine which arm(s) to return
    bool return_left = (arm_selection == "left" || arm_selection == "both");
    bool return_right = (arm_selection == "right" || arm_selection == "both");

    if (!return_left && !return_right) {
        // Invalid arm selection, default to both
        return_left = true;
        return_right = true;
        PLOGW << "Invalid arm selection '" << arm_selection << "', defaulting to 'both'";
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
    }

    return response;
}

// Helper function to create the service callback
static auto create_get_state_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    return [&ros_bridge, research_interface](
        ROSBridgeCallServiceMsg &request, 
        rapidjson::Document::AllocatorType &allocator) {
        
        // Extract arm selection from request arguments
        std::string arm_selection = "both"; // default to both arms
        if (!request.args_json_.IsNull() && request.args_json_.IsObject()) {
            if (request.args_json_.HasMember("arm") && request.args_json_["arm"].IsString()) {
                arm_selection = request.args_json_["arm"].GetString();
            }
        }
        
        PLOGI << "Received service request for /harmony/get_state (arm: " << arm_selection << ")";
        
        // Create response with current joint states and sizes for requested arm
        rapidjson::Document response_data = create_get_state_response(research_interface, arm_selection);
        
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
    // Create service instance
    // Using std_srvs/Trigger which has a string message field we can use for JSON data
    static ROSService get_state_service(ros_bridge, "/harmony/get_state", "std_srvs/Trigger");
    g_get_state_service = &get_state_service;
    
    // Create service callback using helper function
    auto service_callback = create_get_state_callback(ros_bridge, research_interface);

    // Advertise the service
    bool success = get_state_service.Advertise(service_callback);
    if (success) {
        PLOGI << "Advertised service: /harmony/get_state";
    } else {
        PLOGE << "Failed to advertise service: /harmony/get_state";
    }
    return success;
}

bool advertise_get_state_service(ROSBridge& ros_bridge, ResearchInterface* research_interface) {
    if (!g_get_state_service) {
        PLOGW << "Service not initialized, setting up service first";
        return setup_get_state_service(ros_bridge, research_interface);
    }

    // Create the service callback
    auto service_callback = create_get_state_callback(ros_bridge, research_interface);

    // Re-advertise the existing service
    bool success = g_get_state_service->Advertise(service_callback);
    if (success) {
        PLOGI << "Re-advertised service: /harmony/get_state";
    } else {
        PLOGE << "Failed to re-advertise service: /harmony/get_state";
    }
    return success;
}