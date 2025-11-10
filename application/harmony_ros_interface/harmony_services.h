/**
 * @file harmony_services.h
 * @brief ROS service implementations for Harmony research interface
 */

#ifndef HARMONY_SERVICES_H
#define HARMONY_SERVICES_H

#include "research_interface.h"
#include "ros_bridge.h"
#include "ros_service.h"
#include "rapidjson/document.h"

namespace rosbridge2cpp {
    class ROSBridge;
    class ROSService;
}

namespace harmony {
    class ResearchInterface;
}

using namespace rosbridge2cpp;
using namespace harmony;

// Global service instances
extern ROSService* g_get_state_service;
extern ROSService* g_get_state_left_service;
extern ROSService* g_get_state_right_service;
extern ROSService* g_enable_left_service;
extern ROSService* g_enable_right_service;

// Global enable state variables
extern bool g_left_arm_enabled;
extern bool g_right_arm_enabled;


/**
 * @brief Create a callback function for the get_state service
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return Service callback function
 */
auto create_get_state_callback(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Create a response message for the get_state service
 * @param research_interface Pointer to the research interface
 * @param return_left Whether to include left arm data
 * @param return_right Whether to include right arm data
 * @return JSON document containing joint states and sizes for the requested arm(s)
 */
rapidjson::Document create_get_state_response(ResearchInterface* research_interface, bool return_left, bool return_right);

/**
 * @brief Setup and advertise the get_state service (handles both initial setup and re-advertisement)
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_get_state_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Setup and advertise the get_state service for left arm
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_get_state_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Setup and advertise the get_state service for right arm
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_get_state_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Setup and advertise the enable service for left arm
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_enable_left_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Setup and advertise the enable service for right arm
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_enable_right_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

#endif // HARMONY_SERVICES_H

