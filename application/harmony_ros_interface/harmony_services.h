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

// Global service instance
extern ROSService* g_get_state_service;

/**
 * @brief Create a response message for the get_state service
 * @param research_interface Pointer to the research interface
 * @param arm_selection "left", "right", or "both" (default: "both")
 * @return JSON document containing joint states and sizes for the requested arm(s)
 */
rapidjson::Document create_get_state_response(ResearchInterface* research_interface, const std::string& arm_selection = "both");

/**
 * @brief Setup and advertise the get_state service
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully advertised, false otherwise
 */
bool setup_get_state_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

/**
 * @brief Advertise the get_state service (useful after reconnection)
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if service was successfully re-advertised, false otherwise
 */
bool advertise_get_state_service(ROSBridge& ros_bridge, ResearchInterface* research_interface);

#endif // HARMONY_SERVICES_H

