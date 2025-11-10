/**
 * @file message_utils.h
 * @brief Utility functions for creating ROS messages from Harmony data
 */

#ifndef MESSAGE_UTILS_H
#define MESSAGE_UTILS_H

#include "rapidjson/document.h"
#include "joint_states.h"
#include "pose.h"
#include "sizes.h"
#include <string>
#include <array>

using namespace harmony;

/**
 * @brief Create a sensor_msgs/JointState message from joint states
 * @param states Array of joint states
 * @return JSON document representing the joint state message
 */
rapidjson::Document create_joint_state_message(const std::array<JointState, armJointCount>& states);

/**
 * @brief Create a geometry_msgs/TransformStamped message from a pose
 * @param pose The pose to convert
 * @param parent_frame Parent frame ID
 * @param child_frame Child frame ID
 * @return JSON document representing the transform message
 */
rapidjson::Document create_transform(const Pose& pose, const std::string& parent_frame, const std::string& child_frame);

/**
 * @brief Create a static transform message
 * @param parent_frame Parent frame ID
 * @param child_frame Child frame ID
 * @param x, y, z Translation components (in meters)
 * @param qx, qy, qz, qw Quaternion components
 * @return JSON document representing the static transform message
 */
rapidjson::Document create_static_transform(const std::string& parent_frame, const std::string& child_frame, 
                                            double x, double y, double z, double qx, double qy, double qz, double qw);

/**
 * @brief Create a std_msgs/Float64MultiArray message from arm sizes
 * @param sizes Array of arm sizes in mm
 * @return JSON document representing the Float64MultiArray message
 */
rapidjson::Document create_sizes_message(const std::array<double, armSizeCount>& sizes);

#endif // MESSAGE_UTILS_H

