/**
 * @file harmony_topics.h
 * @brief ROS topic subscriber implementations for Harmony research interface
 */

#ifndef HARMONY_TOPICS_H
#define HARMONY_TOPICS_H

#include "research_interface.h"
#include "ros_bridge.h"
#include "ros_topic.h"
#include "harmony_services.h"
#include <array>

namespace rosbridge2cpp {
    class ROSBridge;
    class ROSTopic;
}

namespace harmony {
    class ResearchInterface;
}

using namespace rosbridge2cpp;
using namespace harmony;

// Global subscriber instances
extern ROSTopic* g_left_desired_torque_sub;
extern ROSTopic* g_right_desired_torque_sub;
extern ROSTopic* g_left_desired_stiffness_sub;
extern ROSTopic* g_right_desired_stiffness_sub;
extern ROSTopic* g_left_desired_position_sub;
extern ROSTopic* g_right_desired_position_sub;

/**
 * @brief Update joint override values for an arm
 * @param research_interface Pointer to the research interface
 * @param is_left True for left arm, false for right arm
 * @param torque_values Pointer to array of 7 torque values (nullptr to skip)
 * @param stiffness_values Pointer to array of 7 stiffness values (nullptr to skip)
 * @param position_values Pointer to array of 7 position values (nullptr to skip)
 * @param required_mode The control mode required for this update (torque or impedance)
 * @return true if update was successful, false otherwise
 */
bool updateJointOverride(ResearchInterface* research_interface, bool is_left,
                         const std::array<double, harmony::armJointCount>* torque_values,
                         const std::array<double, harmony::armJointCount>* stiffness_values,
                         const std::array<double, harmony::armJointCount>* position_values,
                         ControlMode required_mode);

/**
 * @brief Setup and subscribe to joint command topics
 * @param ros_bridge Reference to the ROS bridge instance
 * @param research_interface Pointer to the research interface
 * @return true if all subscribers were successfully created, false otherwise
 */
bool setup_joint_command_subscribers(ROSBridge& ros_bridge, ResearchInterface* research_interface);

#endif // HARMONY_TOPICS_H

