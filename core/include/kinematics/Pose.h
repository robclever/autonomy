#ifndef KINEMATICS_POSE_H
#define KINEMATICS_POSE_H

#include <math/Quat.h>
#include <math/Vec3.h>

namespace core
{
namespace kinematics
{

/// @brief Where the aircraft is and how it is oriented in the world.
struct Pose
{
    math::Vec3 position;    // aircraft origin, expressed in NED [m]
    math::Quat orientation; // NED -> body
};

} // namespace kinematics
} // namespace core

#endif // KINEMATICS_POSE_H
