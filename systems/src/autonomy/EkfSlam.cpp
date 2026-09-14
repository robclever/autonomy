#include <autonomy/SLAM/EkfSlam.h>

namespace systems
{
namespace autonomy
{

/// @brief Scaffold: record pose and measurements only. The EKF predict/update
/// equations (state Jacobian, measurement model, data association) are the
/// next implementation step.

void EkfSlam::process(const SlamInput& input)
{
    state_.pose = input.pose;

    /// @brief Naive map update: add any new landmarks by their associated id.
    /// A real EKF would initialize new landmarks from the inverse measurement
    /// model and update the joint covariance.
    for (const auto& m : input.measurements)
    {
        if (m.landmarkId.has_value())
        {
            /// @brief Update existing or append new.
            bool found = false;
            for (auto& lm : state_.landmarks)
            {
                if (lm.id == *m.landmarkId)
                {
                    found = true;
                }
            }
            if (!found)
            {
                SlamLandmark lm;
                lm.id = *m.landmarkId;
                /// @brief Position unknown until triangulated; leave at origin for now.
                lm.positionNed = core::math::Vec3{0.0, 0.0, 0.0};
                state_.landmarks.push_back(lm);
            }
        }
    }
}

SlamState EkfSlam::state() const { return state_; }

void EkfSlam::reset() { state_ = {}; }

} // namespace autonomy
} // namespace systems
