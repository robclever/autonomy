#ifndef AUTONOMY_SLAM_GRAPHSLAM_H
#define AUTONOMY_SLAM_GRAPHSLAM_H

#include <autonomy/SLAM/SlamAlgorithm.h>

namespace systems
{
namespace autonomy
{

// Graph-based SLAM (scaffold).
//
// Graph SLAM poses localization and mapping as a factor graph: nodes are
// vehicle poses and landmark positions, edges are constraints from odometry
// and landmark measurements. A solve step optimizes the whole graph.
//
// This is a structural stub: it accumulates poses and measurements so the
// interface and data flow are exercised. Graph construction and the
// nonlinear least-squares solve are left as the implementation step.
class GraphSlam : public SlamAlgorithm
{
  public:
    GraphSlam() = default;

    void process(const SlamInput& input) override;
    SlamState state() const override;
    void reset() override;

  private:
    SlamState state_;
    // Scaffold storage for the graph structure. A real implementation would
    // build a pose graph + factor graph here.
    std::vector<core::kinematics::Pose> poses_;
};

} // namespace autonomy
} // namespace systems

#endif // AUTONOMY_SLAM_GRAPHSLAM_H
