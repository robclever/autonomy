#include <environment/CameraDevice.h>

namespace sim
{

CameraDevice::CameraDevice(const systems::sensors::camera::CameraSensor& sensor,
                           const sim::ObservationParams& obsParams)
    : sensor_(sensor), obsParams_(obsParams)
{
}

std::vector<scenarios::Detection>
CameraDevice::sense(const World& world, const core::kinematics::Pose& aircraftPose) const
{
    const auto observations = world.observe(aircraftPose, obsParams_);
    const auto detections = sensor_.process(observations);

    /// @brief Convert camera-specific measurements to generic Detections.
    std::vector<scenarios::Detection> out;
    out.reserve(detections.size());
    for (const auto& d : detections)
    {
        scenarios::Detection det;
        det.range = d.range;
        det.azimuth = d.azimuth;
        det.elevation = d.elevation;
        det.confidence = d.detectionConfidence;
        out.push_back(det);
    }
    return out;
}

} // namespace sim
