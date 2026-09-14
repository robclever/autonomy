#ifndef AUTONOMY_SLAM_EKFSLAM_H
#define AUTONOMY_SLAM_EKFSLAM_H

#include <autonomy/SLAM/SlamAlgorithm.h>

namespace systems
{
namespace autonomy
{

// EKF-based SLAM (scaffold).
//
// Extended Kalman Filter SLAM maintains a joint state vector of the vehicle
// pose plus all landmark positions, with a full covariance matrix. The
// prediction step propagates the vehicle pose using motion; the update step
// incorporates landmark measurements.
//
// This is a structural stub: it records the latest pose and measurements so
// the interface and data flow are exercised. The EKF predict/update math is
// left as the first algorithm implementation.
class EkfSlam : public SlamAlgorithm
{
  public:
    EkfSlam() = default;

    void process(const SlamInput& input) override;
    SlamState state() const override;
    void reset() override;

  private:
    SlamState state_;
};

} // namespace autonomy
} // namespace systems

#endif // AUTONOMY_SLAM_EKFSLAM_H
