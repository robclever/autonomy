#ifndef SCENARIOS_SENSORSUITE_H
#define SCENARIOS_SENSORSUITE_H

#include <memory>
#include <vector>

#include <scenarios/Detection.h>
#include <scenarios/ISensorDevice.h>

namespace sim
{
namespace scenarios
{

/// @brief A collection of sensor devices mounted on the ownship.
///
/// Aggregates detections from multiple sensors (radar, camera, ...) into a
/// single vector. This is what the Scenario senses through, so it can field
/// any combination of sensors without knowing which ones are present.
class SensorSuite
{
  public:
    SensorSuite() = default;

    /// @brief Non-copyable (owns unique_ptrs), but movable.
    SensorSuite(const SensorSuite&) = delete;
    SensorSuite& operator=(const SensorSuite&) = delete;
    SensorSuite(SensorSuite&&) = default;
    SensorSuite& operator=(SensorSuite&&) = default;

    /// @brief Add a sensor device to the suite. The suite takes ownership.
    void addDevice(std::unique_ptr<ISensorDevice> device);

    /// @brief Sense the world from all devices and merge the detections.
    std::vector<Detection> sense(const World& world,
                                 const core::kinematics::Pose& aircraftPose) const;

    std::size_t deviceCount() const;

  private:
    std::vector<std::unique_ptr<ISensorDevice>> devices_;
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_SENSORSUITE_H
