#include <scenarios/Trajectory.h>

namespace sim
{
namespace scenarios
{

ListTrajectory::ListTrajectory(std::vector<core::kinematics::Pose> poses) : poses_(std::move(poses))
{
}

void ListTrajectory::addPose(const core::kinematics::Pose& pose) { poses_.push_back(pose); }

core::kinematics::Pose ListTrajectory::poseAt(std::size_t step) const { return poses_.at(step); }

std::size_t ListTrajectory::numSteps() const { return poses_.size(); }

} // namespace scenarios
} // namespace sim
