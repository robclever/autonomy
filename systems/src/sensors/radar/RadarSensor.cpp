#include <sensors/radar/RadarSensor.h>

#include <algorithm>
#include <cmath>
#include <random>

namespace systems
{
namespace sensors
{
namespace radar
{

namespace
{
// Deterministic (seeded) noise so runs are reproducible. A single static
// engine is fine for the scaffold; a per-sensor seeded engine can replace it
// when multiple independent sensors are needed.
std::mt19937& engine()
{
    static std::mt19937 e(12345);
    return e;
}

// Compute detection probability from IR signature strength and range.
// Models a simple 1/r^2 energy falloff: a target is more detectable the
// stronger its signature and the closer it is. Returns a value in [0,1].
double computeDetectibility(double signature, double range)
{
    if (range <= 0.0 || signature <= 0.0)
        return 0.0;
    // Normalize so that signature==1 at range==1m gives detectibility==1,
    // falling off with the square of range.
    const double d = signature / (range * range);
    return std::min(1.0, d);
}
} // namespace

std::vector<RadarEcho> RadarSensor::process(const std::vector<sim::Observation>& observations) const
{
    std::vector<RadarEcho> echoes;
    echoes.reserve(observations.size());

    std::normal_distribution<double> rangeNoise(0.0, params_.rangeNoiseStdDev);
    std::normal_distribution<double> azimuthNoise(0.0, params_.azimuthNoiseStdDev);
    std::normal_distribution<double> elevationNoise(0.0, params_.elevationNoiseStdDev);

    // Use a deterministic per-call seed derived from number of observations so
    // results are stable run-to-run but still vary across calls.
    engine().seed(12345);

    for (const auto& obs : observations)
    {
        // Range gating.
        if (obs.rangeM < params_.minRangeM || obs.rangeM > params_.maxRangeM)
            continue;

        // Detection probability from signature (signalStrength) and range.
        const double detectibility = computeDetectibility(obs.signalStrength, obs.rangeM);
        const double likelihood = detectibility;

        // Threshold: only report detections the radar would actually register.
        if (likelihood < params_.detectionThreshold)
            continue;

        RadarEcho echo;
        echo.range = obs.rangeM + rangeNoise(engine());
        echo.azimuth = obs.azimuthRad + azimuthNoise(engine());
        echo.elevation = obs.elevationRad + elevationNoise(engine());
        echo.signature = obs.signalStrength;
        echo.detectibility = detectibility;
        echo.likelihoodOfDetect = likelihood;
        echoes.push_back(echo);
    }

    return echoes;
}

} // namespace radar
} // namespace sensors
} // namespace systems
