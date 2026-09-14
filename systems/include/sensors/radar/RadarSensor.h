#ifndef SENSORS_RADAR_RADARSENSOR_H
#define SENSORS_RADAR_RADARSENSOR_H

#include <vector>

#include <environment/World.h>
#include <sensors/radar/RadarEcho.h>

namespace systems
{
namespace sensors
{
namespace radar
{

// Tunable parameters for the radar sensor model.
struct RadarParams
{
    double maxRangeM{1000.0};           // ignore targets beyond this range
    double minRangeM{1.0};              // ignore targets closer than this
    double rangeNoiseStdDev{1.0};       // [m] additive noise on range
    double azimuthNoiseStdDev{0.001};   // [rad] additive noise on azimuth
    double elevationNoiseStdDev{0.001}; // [rad] additive noise on elevation
    double detectionThreshold{0.5};     // minimum likelihoodOfDetect to register a hit
};

// Radar sensor model.
//
// Consumes the structured, noiseless observations produced by World::observe()
// and degrades them the way a real radar would:
//   1. Range gating (min/max range)
//   2. Detection probability from IR signature and 1/r^2 falloff
//   3. Additive noise on range / azimuth / elevation
//
// Only targets whose likelihoodOfDetect exceeds the threshold are reported.
// This is the per-sensor model the sim device (RadarDevice) will call into.
class RadarSensor
{
  public:
    explicit RadarSensor(const RadarParams& params = {}) : params_(params) {}

    // Produce radar echoes from true world observations.
    std::vector<RadarEcho> process(const std::vector<sim::Observation>& observations) const;

    const RadarParams& params() const noexcept { return params_; }

  private:
    RadarParams params_;
};

} // namespace radar
} // namespace sensors
} // namespace systems

#endif // SENSORS_RADAR_RADARSENSOR_H
