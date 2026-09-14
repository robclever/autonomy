#ifndef ENVIRONMENT_CAMERADEVICE_H
#define ENVIRONMENT_CAMERADEVICE_H

#include <vector>

#include <environment/World.h>
#include <kinematics/Pose.h>
#include <scenarios/Detection.h>
#include <scenarios/ISensorDevice.h>
#include <sensors/camera/CameraSensor.h>

namespace sim
{

/// @brief A camera device mounted on the ownship (player aircraft).
///
/// Implements ISensorDevice so it can be added to a SensorSuite alongside any
/// other sensor. Internally it queries World::observe() for true observations
/// and feeds them through the CameraSensor model (range gating, FOV gating,
/// contrast/brightness detection, noise) to produce generic Detections.
class CameraDevice : public scenarios::ISensorDevice
{
  public:
    explicit CameraDevice(const systems::sensors::camera::CameraSensor& sensor,
                          const sim::ObservationParams& obsParams = {});

    /// @brief Returns generic Detections (body-frame spherical + confidence).
    std::vector<scenarios::Detection>
    sense(const World& world, const core::kinematics::Pose& aircraftPose) const override;

    const systems::sensors::camera::CameraSensor& sensor() const noexcept { return sensor_; }

  private:
    const systems::sensors::camera::CameraSensor& sensor_;
    sim::ObservationParams obsParams_;
};

} // namespace sim

#endif // ENVIRONMENT_CAMERADEVICE_H
