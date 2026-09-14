#ifndef SENSORS_CAMERA_CAMERASENSOR_H
#define SENSORS_CAMERA_CAMERASENSOR_H

#include <vector>

#include <environment/World.h>
#include <sensors/camera/CameraMeasurement.h>

namespace systems
{
namespace sensors
{
namespace camera
{

/// @brief Tunable parameters for the camera sensor model.
struct CameraParams
{
    double maxRangeM{500.0};               // max detection range
    double minRangeM{1.0};                 // min detection range
    double maxAzimuthFOVRad{M_PI / 3.0};   // half-angle azimuth field of view (each side)
    double maxElevationFOVRad{M_PI / 6.0}; // half-angle elevation field of view
    double rangeNoiseStdDev{0.5};          // [m] additive noise on range
    double azimuthNoiseStdDev{0.0005};     // [rad] additive noise on azimuth
    double elevationNoiseStdDev{0.0005};   // [rad] additive noise on elevation
    double detectionThreshold{0.3};        // minimum detectionConfidence to register
};

/// @brief Camera sensor model.
///
/// Consumes the structured, noiseless observations produced by World::observe()
/// and degrades them the way a real camera would:
///   1. Range gating (min/max range)
///   2. Field-of-view gating (azimuth and elevation within FOV)
///   3. Detection probability from contrast/brightness and range falloff
///   4. Additive noise on range / azimuth / elevation
///
/// Only targets whose detectionConfidence exceeds the threshold are reported.
class CameraSensor
{
  public:
    explicit CameraSensor(const CameraParams& params = {}) : params_(params) {}

    /// @brief Produce camera detections from true world observations.
    std::vector<CameraMeasurement> process(const std::vector<sim::Observation>& observations) const;

    const CameraParams& params() const noexcept { return params_; }

  private:
    CameraParams params_;
};

} // namespace camera
} // namespace sensors
} // namespace systems

#endif // SENSORS_CAMERA_CAMERASENSOR_H
