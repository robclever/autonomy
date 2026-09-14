#include <sensors/camera/CameraSensor.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace systems
{
namespace sensors
{
namespace camera
{

namespace
{
// Deterministic (seeded) noise so runs are reproducible.
std::mt19937& engine()
{
    static std::mt19937 e(54321);
    return e;
}

// Compute detection confidence from contrast, brightness, and range.
// Models resolution falloff: a target is more confidently detected the
// higher its contrast/brightness and the closer it is. Returns [0,1].
double computeConfidence(double contrast, double brightness, double range)
{
    if (range <= 0.0 || contrast <= 0.0 || brightness <= 0.0)
        return 0.0;
    // Normalize so contrast*brightness==1 at range==1m gives confidence==1,
    // falling off with the square of range.
    const double c = (contrast * brightness) / (range * range);
    return std::min(1.0, c);
}
} // namespace

std::vector<CameraMeasurement>
CameraSensor::process(const std::vector<sim::Observation>& observations) const
{
    std::vector<CameraMeasurement> detections;
    detections.reserve(observations.size());

    std::normal_distribution<double> rangeNoise(0.0, params_.rangeNoiseStdDev);
    std::normal_distribution<double> azimuthNoise(0.0, params_.azimuthNoiseStdDev);
    std::normal_distribution<double> elevationNoise(0.0, params_.elevationNoiseStdDev);

    engine().seed(54321);

    for (const auto& obs : observations)
    {
        // Range gating.
        if (obs.rangeM < params_.minRangeM || obs.rangeM > params_.maxRangeM)
            continue;

        // Field-of-view gating.
        if (std::fabs(obs.azimuthRad) > params_.maxAzimuthFOVRad)
            continue;
        if (std::fabs(obs.elevationRad) > params_.maxElevationFOVRad)
            continue;

        // Detection confidence from contrast/brightness (signalStrength) and range.
        const double confidence =
            computeConfidence(obs.signalStrength, obs.signalStrength, obs.rangeM);

        if (confidence < params_.detectionThreshold)
            continue;

        CameraMeasurement det;
        det.range = obs.rangeM + rangeNoise(engine());
        det.azimuth = obs.azimuthRad + azimuthNoise(engine());
        det.elevation = obs.elevationRad + elevationNoise(engine());
        det.contrast = obs.signalStrength;
        det.brightness = obs.signalStrength;
        det.detectionConfidence = confidence;
        detections.push_back(det);
    }

    return detections;
}

} // namespace camera
} // namespace sensors
} // namespace systems
