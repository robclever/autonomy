#include <scenarios/MeasurementAdapter.h>

#include <cmath>
#include <limits>

namespace sim
{
namespace scenarios
{

namespace
{
// Compute range/azimuth/elevation from aircraft pose to a landmark (NED).
// Mirrors World::observe() so we can match echoes to ground truth.
struct RazEl
{
    double range;
    double azimuth;
    double elevation;
};

RazEl toRazEl(const Landmark& lm, const core::kinematics::Pose& pose)
{
    const core::math::Vec3 rel = lm.positionNed - pose.position;
    const double range = rel.norm();
    const core::kinematics::Transform nedToBody{pose.position, pose.orientation};
    const core::math::Vec3 dir = nedToBody.applyToDirection(rel / range);
    return {range, std::atan2(dir.y, dir.x), std::asin(std::max(-1.0, std::min(1.0, -dir.z)))};
}
} // namespace

std::vector<systems::autonomy::SlamMeasurement>
MeasurementAdapter::convert(const std::vector<Detection>& detections)
{
    std::vector<systems::autonomy::SlamMeasurement> out;
    out.reserve(detections.size());
    for (const auto& d : detections)
    {
        systems::autonomy::SlamMeasurement m;
        m.range = d.range;
        m.azimuth = d.azimuth;
        m.elevation = d.elevation;
        // landmarkId left unset — unknown association.
        out.push_back(m);
    }
    return out;
}

std::vector<systems::autonomy::SlamMeasurement>
MeasurementAdapter::convertAssociated(const std::vector<Detection>& detections,
                                      const std::vector<Landmark>& landmarks,
                                      const core::kinematics::Pose& aircraftPose, double toleranceM)
{
    // Precompute ground-truth range/az/el to each landmark.
    std::vector<RazEl> truth;
    truth.reserve(landmarks.size());
    for (const auto& lm : landmarks)
        truth.push_back(toRazEl(lm, aircraftPose));

    std::vector<systems::autonomy::SlamMeasurement> out;
    out.reserve(detections.size());
    for (const auto& d : detections)
    {
        systems::autonomy::SlamMeasurement m;
        m.range = d.range;
        m.azimuth = d.azimuth;
        m.elevation = d.elevation;

        // Nearest-neighbor in range; require match within tolerance.
        double bestErr = std::numeric_limits<double>::infinity();
        std::size_t bestIdx = 0;
        for (std::size_t i = 0; i < landmarks.size(); ++i)
        {
            const double err = std::fabs(truth[i].range - d.range);
            if (err < bestErr)
            {
                bestErr = err;
                bestIdx = i;
            }
        }
        if (bestErr <= toleranceM)
            m.landmarkId = landmarks[bestIdx].id;

        out.push_back(m);
    }
    return out;
}

} // namespace scenarios
} // namespace sim
