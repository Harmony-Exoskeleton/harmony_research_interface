/** @file
 * @brief Declares Pose struct.
 *
 */

#ifndef HARMONY_POSE_H
#define HARMONY_POSE_H

namespace harmony {

struct Point {
    double x, y, z;
};

struct Quaternion {
    double x, y, z, w;
};

struct Pose {
    Point position_mm;
    Quaternion orientation;
};

struct Poses {
    Pose leftEndEffector;
    Pose rightEndEffector;
};

} // namespace harmony

#endif