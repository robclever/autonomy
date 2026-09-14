#ifndef AUTONOMY_SLAM_SLAMSTATE_H
#define AUTONOMY_SLAM_SLAMSTATE_H

#include <cstdint>
#include <vector>

#include <kinematics/Pose.h>
#include <math/Vec3.h>

namespace systems
{
namespace autonomy
{

// A landmark as estimated by SLAM: position in NED plus an identifier.
// The scaffold stores only the mean; a full implementation adds a
// covariance block per landmark.
struct SlamLandmark
{
    std::uint32_t id{0};
    core::math::Vec3 positionNed; // estimated position [m], NED
};

// The complete output of a SLAM algorithm at one time step: the vehicle's
// estimated pose and the current map of landmarks. This is what autonomy,
// planning, and evaluation consume.
struct SlamState
{
    core::kinematics::Pose pose;         // vehicle pose estimate (NED)
    std::vector<SlamLandmark> landmarks; // estimated landmark positions
};

} // namespace autonomy
} // namespace systems

#endif // AUTONOMY_SLAM_SLAMSTATE_H
