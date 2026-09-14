#ifndef SENSORS_CAMERA_CAMERAMEASUREMENT_H
#define SENSORS_CAMERA_CAMERAMEASUREMENT_H

#include <measurement/Spherical.h>

namespace systems
{
namespace sensors
{
namespace camera
{

// A single camera detection. Extends the base Spherical measurement (range,
// azimuth, elevation) with camera-specific parameters governing how the
// target is detected by an optical sensor.
//
// Camera detects targets by their visual contrast and brightness (illuminosity)
// against the background, rather than by radar cross-section / IR signature.
// A target is more detectable the higher its contrast and brightness, and
// the closer it is (resolution falls off with range).
struct CameraMeasurement : public core::measurement::Spherical
{
    double contrast{0.0};            // [0,1]: contrast against background
    double brightness{0.0};          // illuminosity of the target
    double detectionConfidence{0.0}; // [0,1]: probability the camera registers this
};

} // namespace camera
} // namespace sensors
} // namespace systems

#endif // SENSORS_CAMERA_CAMERAMEASUREMENT_H
