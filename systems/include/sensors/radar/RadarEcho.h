#ifndef SENSORS_RADAR_RADARECHO_H
#define SENSORS_RADAR_RADARECHO_H

#include <measurement/Spherical.h>

namespace systems
{
namespace sensors
{
namespace radar
{

// A single radar detection. Extends the base Spherical measurement (range,
// azimuth, elevation) with radar-specific parameters that govern how the
// target is detected and how likely it is to be seen.
//
// Radar uses IR signatures (not illuminosity) to detect targets. A target's
// signature, combined with range, determines how detectable it is and the
// probability that the radar actually registers a detection.
struct RadarEcho : public core::measurement::Spherical
{
    double signature{0.0};          // IR signature strength (dimensionless, >= 0)
    double detectibility{0.0};      // [0,1]: how detectable this target is given its signature
    double likelihoodOfDetect{0.0}; // [0,1]: probability the radar registers this detection
};

} // namespace radar
} // namespace sensors
} // namespace systems

#endif // SENSORS_RADAR_RADARECHO_H
