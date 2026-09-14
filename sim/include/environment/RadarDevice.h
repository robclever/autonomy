#ifndef ENVIRONMENT_RADARDEVICE_H
#define ENVIRONMENT_RADARDEVICE_H

#include <vector>

#include <environment/World.h>
#include <kinematics/Pose.h>
#include <scenarios/Detection.h>
#include <scenarios/ISensorDevice.h>
#include <sensors/radar/RadarEcho.h>
#include <sensors/radar/RadarSensor.h>

namespace sim
{

// A radar device mounted on the ownship (player aircraft).
//
// Implements ISensorDevice so it can be added to a SensorSuite alongside any
// other sensor. Internally it queries World::observe() for true observations
// and feeds them through the RadarSensor model (range gating, IR-signature
// detection, noise) to produce radar echoes or generic Detections.
//
// Multiple devices (different sensors, different mount poses) can be added to
// the same ownship.
class RadarDevice : public scenarios::ISensorDevice
{
  public:
    explicit RadarDevice(const systems::sensors::radar::RadarSensor& sensor,
                         const sim::ObservationParams& obsParams = {});

    // Radar-specific output: returns the raw radar echoes.
    std::vector<systems::sensors::radar::RadarEcho>
    senseEchoes(const World& world, const core::kinematics::Pose& aircraftPose) const;

    // ISensorDevice override — returns generic Detections for the SensorSuite.
    std::vector<scenarios::Detection>
    sense(const World& world, const core::kinematics::Pose& aircraftPose) const override;

    const systems::sensors::radar::RadarSensor& sensor() const noexcept { return sensor_; }

  private:
    const systems::sensors::radar::RadarSensor& sensor_;
    sim::ObservationParams obsParams_;
};

} // namespace sim

#endif // ENVIRONMENT_RADARDEVICE_H
