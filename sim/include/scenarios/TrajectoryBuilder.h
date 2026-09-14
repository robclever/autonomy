#ifndef SCENARIOS_TRAJECTORYBUILDER_H
#define SCENARIOS_TRAJECTORYBUILDER_H

#include <vector>

#include <kinematics/Pose.h>
#include <math/Vec3.h>

namespace sim
{
namespace scenarios
{

// Builds an aircraft trajectory from flight segments using a simple kinematic
// model. Maintains a position + heading + pitch state and integrates forward
// in fixed timesteps.
//
// Usage:
//   TrajectoryBuilder builder({0,0,-500}, 0.0);  // start NED, heading 0 (north)
//   builder.flyStraight(1000.0, 50.0, 0.1);      // 1000 m at 50 m/s, dt=0.1
//   builder.turnTo(M_PI/2, 0.05, 0.1);            // turn east at 0.05 rad/s
//   builder.flyStraight(500.0, 50.0, 0.1);
//   auto traj = builder.build();
class TrajectoryBuilder
{
  public:
    // startPos: NED [m]. startYaw: rad (0 = north, +east). startPitch: rad.
    TrajectoryBuilder(const core::math::Vec3& startPos, double startYaw = 0.0,
                      double startPitch = 0.0);

    // Fly straight at current heading/pitch for the given distance [m] at
    // constant speed [m/s], integrating with timestep dt [s].
    TrajectoryBuilder& flyStraight(double distanceM, double speedMps, double dt);

    // Turn at a constant rate [rad/s] for the given angle [rad], while
    // continuing to move forward at speed [m/s]. Positive angle = right turn
    // (clockwise viewed from above, i.e. increasing yaw in NED).
    TrajectoryBuilder& turnBy(double angleRad, double turnRateRps, double speedMps, double dt);

    // Turn from the current heading to a target heading at a constant rate.
    TrajectoryBuilder& turnTo(double targetYawRad, double turnRateRps, double speedMps, double dt);

    // Climb/descend to a target altitude while moving forward. targetAltM is
    // height above ground (positive up); the builder converts to NED down.
    TrajectoryBuilder& climbTo(double targetAltM, double climbRateMps, double speedMps, double dt);

    // Return the accumulated poses (one per timestep, including the start).
    std::vector<core::kinematics::Pose> build() const;

    const core::math::Vec3& position() const { return pos_; }
    double yaw() const { return yaw_; }
    double pitch() const { return pitch_; }

  private:
    // Integrate one timestep forward at the current heading/pitch and speed,
    // appending the new pose.
    void step(double speedMps, double dt);
    void appendCurrent();

    core::math::Vec3 pos_;
    double yaw_{0.0};
    double pitch_{0.0};
    std::vector<core::kinematics::Pose> poses_;
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_TRAJECTORYBUILDER_H
