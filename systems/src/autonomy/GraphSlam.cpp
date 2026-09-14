#include <autonomy/SLAM/GraphSlam.h>

namespace systems
{
namespace autonomy
{

// Scaffold: accumulate poses and record measurements only. Building the
// factor graph and running the optimization solve is the next step.

void GraphSlam::process(const SlamInput& input)
{
    state_.pose = input.pose;
    poses_.push_back(input.pose);

    for (const auto& m : input.measurements)
    {
        if (!m.landmarkId.has_value())
            continue;
        bool found = false;
        for (auto& lm : state_.landmarks)
        {
            if (lm.id == *m.landmarkId)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            SlamLandmark lm;
            lm.id = *m.landmarkId;
            lm.positionNed = core::math::Vec3{0.0, 0.0, 0.0};
            state_.landmarks.push_back(lm);
        }
    }
}

SlamState GraphSlam::state() const { return state_; }

void GraphSlam::reset()
{
    state_ = {};
    poses_.clear();
}

} // namespace autonomy
} // namespace systems
