#include <environment/RadarDevice.h>

namespace sim
{

RadarDevice::RadarDevice(const systems::sensors::radar::RadarSensor& sensor,
                         const sim::ObservationParams& obsParams)
    : sensor_(sensor), obsParams_(obsParams)
{
}

std::vector<systems::sensors::radar::RadarEcho>
RadarDevice::senseEchoes(const World& world, const core::kinematics::Pose& aircraftPose) const
{
    const auto observations = world.observe(aircraftPose, obsParams_);
    return sensor_.process(observations);
}

std::vector<scenarios::Detection>
RadarDevice::sense(const World& world, const core::kinematics::Pose& aircraftPose) const
{
    const auto echoes = senseEchoes(world, aircraftPose);

    // Convert radar-specific echoes to generic Detections.
    std::vector<scenarios::Detection> out;
    out.reserve(echoes.size());
    for (const auto& e : echoes)
    {
        scenarios::Detection det;
        det.range = e.range;
        det.azimuth = e.azimuth;
        det.elevation = e.elevation;
        det.confidence = e.likelihoodOfDetect;
        out.push_back(det);
    }
    return out;
}

} // namespace sim
