#include <environment/World.h>

#include <cmath>
#include <utility>

namespace sim
{

std::uint32_t World::addLandmark(const std::string& name, const core::math::Vec3& positionNed,
                                 double reflectivity)
{
    Landmark lm;
    lm.id = nextId_++;
    lm.name = name;
    lm.positionNed = positionNed;
    lm.reflectivity = reflectivity;
    landmarks_.push_back(std::move(lm));
    return lm.id;
}

std::optional<Landmark> World::landmark(std::uint32_t id) const
{
    for (const auto& lm : landmarks_)
    {
        if (lm.id == id)
            return lm;
    }
    return std::nullopt;
}

std::vector<Observation> World::observe(const core::kinematics::Pose& observer,
                                        const ObservationParams& params) const
{
    std::vector<Observation> out;
    const core::kinematics::Transform nedToBody{observer.position, observer.orientation};

    for (const auto& lm : landmarks_)
    {
        const core::math::Vec3 rel = lm.positionNed - observer.position;
        const double range = rel.norm();
        if (range < params.minRangeM || range > params.maxRangeM)
            continue;

        const core::math::Vec3 dirBody =
            nedToBody.applyToDirection(rel / range); // unit vector, body frame

        Observation obs;
        obs.id = lm.id;
        obs.rangeM = range;
        obs.azimuthRad = std::atan2(dirBody.y, dirBody.x);
        obs.elevationRad = std::asin(-dirBody.z); // + up, since body z is down
        obs.directionBody = dirBody;
        obs.positionNed = lm.positionNed;
        obs.signalStrength = lm.reflectivity / (range * range);
        out.push_back(obs);
    }
    return out;
}

} // namespace sim