#ifndef SCENARIOS_DETECTION_H
#define SCENARIOS_DETECTION_H

#include <measurement/Spherical.h>

namespace sim
{
namespace scenarios
{

// A generic detection: the common output type of every sensor device.
//
// All sensors (radar, camera, ...) ultimately report what they see as a set
// of detections in spherical coordinates (range, azimuth, elevation) with a
// confidence. This type is the single currency that flows from sensors,
// through the SensorSuite, into the MeasurementAdapter and on to SLAM. It
// lets the rest of the pipeline be sensor-agnostic.
struct Detection : public core::measurement::Spherical
{
    double confidence{0.0}; // [0,1] detection confidence / quality
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_DETECTION_H
