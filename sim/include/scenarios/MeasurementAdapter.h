#ifndef SCENARIOS_MEASUREMENTADAPTER_H
#define SCENARIOS_MEASUREMENTADAPTER_H

#include <vector>

#include <Landmark.h>
#include <autonomy/SLAM/SlamMeasurement.h>
#include <kinematics/Pose.h>
#include <kinematics/Transform.h>
#include <scenarios/Detection.h>

namespace sim
{
namespace scenarios
{

/// @brief Converts sensor detections into SLAM measurements (algorithm input).
///
/// Handles the coordinate/typing bridge from the generic Detection type to
/// SlamMeasurement, and optionally performs ground-truth data association for
/// testing. Because it operates on Detections (not sensor-specific types), it
/// works for any sensor — radar, camera, or otherwise.
class MeasurementAdapter
{
  public:
    /// @brief Convert detections to measurements with unknown association (no landmarkId).
    /// Just copies the spherical fields; SLAM must associate on its own.
    static std::vector<systems::autonomy::SlamMeasurement>
    convert(const std::vector<Detection>& detections);

    /// @brief Convert with ground-truth data association. For each detection, finds the
    /// landmark whose true bearing/range from the aircraft pose best matches
    /// the detection. If a match is within toleranceM, the measurement's
    /// landmarkId is set; otherwise it is left unknown. This lets tests exercise
    /// the SLAM pipeline with known correspondences.
    static std::vector<systems::autonomy::SlamMeasurement>
    convertAssociated(const std::vector<Detection>& detections,
                      const std::vector<Landmark>& landmarks,
                      const core::kinematics::Pose& aircraftPose, double toleranceM = 75.0);
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_MEASUREMENTADAPTER_H
