#ifndef SCENARIOS_SCENARIO_H
#define SCENARIOS_SCENARIO_H

#include <cstddef>
#include <vector>

#include <autonomy/SLAM/SlamAlgorithm.h>
#include <environment/World.h>
#include <kinematics/Pose.h>
#include <scenarios/Detection.h>
#include <scenarios/MeasurementAdapter.h>
#include <scenarios/SensorSuite.h>
#include <scenarios/Trajectory.h>

namespace sim
{
namespace scenarios
{

/// @brief Recorded result of one simulation step. Lets tests inspect what happened
/// at each timestep: the true pose, what the sensors saw (as generic
/// detections), what was fed to SLAM, and SLAM's state before/after.
struct ScenarioStep
{
    std::size_t step{0};
    double timeS{0.0};
    core::kinematics::Pose truePose;
    std::vector<Detection> detections;
    std::vector<systems::autonomy::SlamMeasurement> measurements;
    systems::autonomy::SlamState slamBefore;
    systems::autonomy::SlamState slamAfter;
};

/// @brief A simulation scenario: a world (landmarks), an aircraft trajectory, a suite
/// of sensor devices, and a SLAM algorithm. Steps through time, at each step
/// sensing the world through all sensors and feeding merged measurements to SLAM.
///
/// The scenario owns the world, trajectory, and sensor suite; the SLAM
/// algorithm is referenced so the caller can inspect/swap it.
class Scenario
{
  public:
    Scenario(World world, std::unique_ptr<Trajectory> trajectory, SensorSuite sensors,
             systems::autonomy::SlamAlgorithm& slam, double dt = 0.1);

    /// @brief Whether to use ground-truth data association when converting detections.
    void setUseAssociation(bool useAssoc);

    /// @brief Run all remaining steps to the end of the trajectory.
    void run();

    /// @brief Run a single step. No-op if already at the end.
    void step();

    /// @brief Reset to the beginning (clears recorded steps and resets SLAM).
    void reset();

    /// @brief Access recorded steps.
    const std::vector<ScenarioStep>& steps() const;
    std::size_t currentStep() const;
    bool atEnd() const;

  private:
    World world_;
    std::unique_ptr<Trajectory> trajectory_;
    SensorSuite sensors_;
    systems::autonomy::SlamAlgorithm& slam_;
    double dt_;
    bool useAssoc_{false};
    std::vector<ScenarioStep> steps_;
    std::size_t cursor_{0};
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_SCENARIO_H
