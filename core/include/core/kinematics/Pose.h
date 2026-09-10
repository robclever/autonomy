#pragma once

#include "core/math/Quat.h"
#include "core/math/Vec3.h"

namespace core {
namespace kinematics {

// Where the aircraft is and how it is oriented in the world.
struct Pose
{
    math::Vec3 position;    // aircraft origin, expressed in NED [m]
    math::Quat orientation; // NED -> body
};

} // namespace kinematics
} // namespace core
