#ifndef SCENARIOS_ISENSORDEVICE_H
#define SCENARIOS_ISENSORDEVICE_H

#include <vector>

#include <environment/World.h>
#include <kinematics/Pose.h>
#include <scenarios/Detection.h>

namespace sim
{
namespace scenarios
{

/// @brief Common interface for any sensor device mounted on the ownship.
///
/// A sensor device wraps a sensor model and knows how to query the world for
/// true observations, run them through its model, and return a set of
/// detections. Radar, camera, infrared, etc. all implement this interface so
/// a SensorSuite can aggregate them and the Scenario can stay sensor-agnostic.
class ISensorDevice
{
  public:
    virtual ~ISensorDevice() = default;

    /// @brief Sense the world from the given aircraft pose. Returns the detections
    /// this sensor would register (already in body-frame spherical coordinates).
    virtual std::vector<Detection> sense(const World& world,
                                         const core::kinematics::Pose& aircraftPose) const = 0;
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_ISENSORDEVICE_H
