#ifndef MEASUREMENT_SPHERICAL_H
#define MEASUREMENT_SPHERICAL_H

namespace core
{
namespace measurement
{

// Base spherical measurement: range, azimuth, elevation.
//
// This is the fundamental geometric measurement type for any sensor that
// reports targets in spherical coordinates (radar, infrared, etc.). Sensor-
// specific measurements (e.g. RadarEcho) extend it with modality-specific
// parameters such as signature and detection probability.
struct Spherical
{
    double range{0.0};     // [m] distance to target
    double azimuth{0.0};   // [rad]; 0 straight ahead, + to the right (body y)
    double elevation{0.0}; // [rad]; positive above the aircraft
};

} // namespace measurement
} // namespace core

#endif // MEASUREMENT_SPHERICAL_H
