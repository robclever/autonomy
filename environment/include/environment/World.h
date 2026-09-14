#ifndef ENVIRONMENT_WORLD_H
#define ENVIRONMENT_WORLD_H

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

#include <Landmark.h>
#include <kinematics/Pose.h>
#include <kinematics/Transform.h>
#include <math/Vec3.h>

namespace sim
{

// One structured observation returned by World::observe(): what the
// environment would report for a single landmark seen from an aircraft pose.
// It carries the range-bearing form a real sensor pipeline will consume,
// plus ground truth for evaluation.
struct Observation
{
    std::uint32_t id{0};
    double rangeM{0.0};
    double azimuthRad{0.0};         // radians; 0 straight ahead, + y (right)
    double elevationRad{0.0};       // radians; positive above the aircraft
    core::math::Vec3 directionBody; // unit vector, body frame
    core::math::Vec3 positionNed;   // ground truth (for evaluation)
    double signalStrength{0.0};
};

struct ObservationParams
{
    double maxRangeM{1000.0};
    double minRangeM{1.0};
};

// The world model: it owns the truth. Given an aircraft pose it emits exactly
// what a noiseless "ideal sensor" would observe. Per-sensor models in
// systems/sensors/ will later degrade these observations with noise/bias.
class World
{
  public:
    std::uint32_t addLandmark(const std::string& name, const core::math::Vec3& positionNed,
                              double reflectivity = 1.0);

    const std::vector<Landmark>& landmarks() const noexcept { return landmarks_; }
    std::optional<Landmark> landmark(std::uint32_t id) const;

    std::vector<Observation> observe(const core::kinematics::Pose& observer,
                                     const ObservationParams& params = {}) const;

  private:
    std::vector<Landmark> landmarks_;
    std::uint32_t nextId_{1};
};

} // namespace sim

#endif // ENVIRONMENT_WORLD_H
