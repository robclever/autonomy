#ifndef AUTONOMY_SLAM_SLAMALGORITHM_H
#define AUTONOMY_SLAM_SLAMALGORITHM_H

#include <vector>

#include <autonomy/SLAM/SlamMeasurement.h>
#include <autonomy/SLAM/SlamState.h>
#include <kinematics/Pose.h>

namespace systems
{
namespace autonomy
{

/// @brief One time-step of input to a SLAM algorithm.
///
///   pose         — the vehicle's pose for this step (from platform / fusion).
///   measurements — landmark detections (range/azimuth/elevation) from the
///                  sensor pipeline, with optional data association.
///
/// A SLAM implementation ingests this and produces/updates a SlamState.
struct SlamInput
{
    core::kinematics::Pose pose;
    std::vector<SlamMeasurement> measurements;
};

/// @brief Abstract interface for SLAM algorithms.
///
/// Swap implementations (EKF, graph, particle-filter, ...) behind this
/// interface so the rest of the autonomy stack and the sim are agnostic to
/// the specific algorithm. The contract:
///   - process()  : ingest one time-step and update the internal state.
///   - state()    : read the current estimate (pose + map).
///   - reset()    : clear all state back to initialization.
class SlamAlgorithm
{
  public:
    virtual ~SlamAlgorithm() = default;

    /// @brief Ingest one time-step of measurements and update the estimate.
    virtual void process(const SlamInput& input) = 0;

    /// @brief Return the current SLAM estimate (pose + map).
    virtual SlamState state() const = 0;

    /// @brief Reset all internal state.
    virtual void reset() = 0;
};

} // namespace autonomy
} // namespace systems

#endif // AUTONOMY_SLAM_SLAMALGORITHM_H
