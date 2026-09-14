#ifndef SCENARIOS_TRAJECTORY_H
#define SCENARIOS_TRAJECTORY_H

#include <cstddef>
#include <vector>

#include <kinematics/Pose.h>

namespace sim
{
namespace scenarios
{

// An aircraft trajectory: a sequence of poses indexed by timestep.
//
// Produce one pose per simulation step so a Scenario can advance the
// ownship through the world. Concrete implementations generate poses from
// a kinematic model (TrajectoryBuilder) or an explicit list.
class Trajectory
{
  public:
    virtual ~Trajectory() = default;

    // Pose at the given timestep. Must hold for step < numSteps().
    virtual core::kinematics::Pose poseAt(std::size_t step) const = 0;

    // Total number of steps in the trajectory.
    virtual std::size_t numSteps() const = 0;
};

// A trajectory backed by an explicit list of poses.
class ListTrajectory : public Trajectory
{
  public:
    explicit ListTrajectory(std::vector<core::kinematics::Pose> poses);
    ListTrajectory() = default;

    void addPose(const core::kinematics::Pose& pose);

    core::kinematics::Pose poseAt(std::size_t step) const override;
    std::size_t numSteps() const override;

  private:
    std::vector<core::kinematics::Pose> poses_;
};

} // namespace scenarios
} // namespace sim

#endif // SCENARIOS_TRAJECTORY_H
