#include <scenarios/TrajectoryBuilder.h>

#include <cmath>

#include <math/Quat.h>

namespace sim
{
namespace scenarios
{

TrajectoryBuilder::TrajectoryBuilder(const core::math::Vec3& startPos, double startYaw,
                                     double startPitch)
    : pos_(startPos), yaw_(startYaw), pitch_(startPitch)
{
    appendCurrent();
}

void TrajectoryBuilder::appendCurrent()
{
    poses_.push_back({pos_, core::math::Quat::fromYawPitchRoll(yaw_, pitch_, 0.0)});
}

void TrajectoryBuilder::step(double speedMps, double dt)
{
    /// @brief Body forward axis is (1,0,0). Rotate by pitch about body-y, then yaw
    /// about world-down (NED z). In NED: forward = (cos yaw cos pitch,
    /// sin yaw cos pitch, -sin pitch).
    const double cn = std::cos(yaw_) * std::cos(pitch_);
    const double ce = std::sin(yaw_) * std::cos(pitch_);
    const double cd = -std::sin(pitch_);
    pos_.x += cn * speedMps * dt;
    pos_.y += ce * speedMps * dt;
    pos_.z += cd * speedMps * dt;
    appendCurrent();
}

TrajectoryBuilder& TrajectoryBuilder::flyStraight(double distanceM, double speedMps, double dt)
{
    const double stepDist = speedMps * dt;
    const std::size_t n = static_cast<std::size_t>(std::ceil(distanceM / stepDist));
    for (std::size_t i = 0; i < n; ++i)
    {
        step(speedMps, dt);
    }
    return *this;
}

TrajectoryBuilder& TrajectoryBuilder::turnBy(double angleRad, double turnRateRps, double speedMps,
                                             double dt)
{
    const double turnDir = (angleRad >= 0.0) ? 1.0 : -1.0;
    const double rate = turnRateRps * turnDir;
    double turned = 0.0;
    const double target = std::fabs(angleRad);
    while (turned < target)
    {
        yaw_ += rate * dt;
        step(speedMps, dt);
        turned += turnRateRps * dt;
    }
    return *this;
}

TrajectoryBuilder& TrajectoryBuilder::turnTo(double targetYawRad, double turnRateRps,
                                             double speedMps, double dt)
{
    double delta = targetYawRad - yaw_;
    /// @brief Normalize to [-pi, pi].
    while (delta > M_PI)
    {
        delta -= 2.0 * M_PI;
    }
    while (delta < -M_PI)
    {
        delta += 2.0 * M_PI;
    }
    return turnBy(delta, turnRateRps, speedMps, dt);
}

TrajectoryBuilder& TrajectoryBuilder::climbTo(double targetAltM, double climbRateMps,
                                              double speedMps, double dt)
{
    /// @brief targetAltM is height above ground (positive up); NED down = -alt.
    const double targetDown = -targetAltM;
    const double downChange = targetDown - pos_.z; // positive = descend
    const double dir = (downChange >= 0.0) ? 1.0 : -1.0;
    double climbed = 0.0;
    const double target = std::fabs(downChange);
    while (climbed < target)
    {
        /// @brief Climb changes altitude; integrate vertical motion directly.
        pos_.z += dir * climbRateMps * dt;
        /// @brief Also advance horizontally at current heading (level forward).
        const double cn = std::cos(yaw_);
        const double ce = std::sin(yaw_);
        pos_.x += cn * speedMps * dt;
        pos_.y += ce * speedMps * dt;
        appendCurrent();
        climbed += climbRateMps * dt;
    }
    return *this;
}

std::vector<core::kinematics::Pose> TrajectoryBuilder::build() const { return poses_; }

} // namespace scenarios
} // namespace sim
