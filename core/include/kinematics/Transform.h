#ifndef KINEMATICS_TRANSFORM_H
#define KINEMATICS_TRANSFORM_H

#include <kinematics/Pose.h>
#include <math/Quat.h>
#include <math/Vec3.h>

namespace core
{
namespace kinematics
{

/// @brief Transform from a source frame A to a target frame B.
/// A Transform is fully described by where B's origin sits in A, plus the
/// A -> B rotation. Use `Transform::nedToBody` for the primary world->body map.
struct Transform
{
    math::Vec3 translation{0.0, 0.0, 0.0};   // B's origin, expressed in A
    math::Quat rotation{1.0, 0.0, 0.0, 0.0}; // A -> B

    /// @brief Point/direction from A into B.
    math::Vec3 applyToPoint(const math::Vec3& p) const { return rotation.rotate(p - translation); }
    math::Vec3 applyToDirection(const math::Vec3& d) const { return rotation.rotate(d); }

    /// @brief Point/direction from B back into A.
    math::Vec3 applyInverseToPoint(const math::Vec3& p) const
    {
        return rotation.inverse().rotate(p) + translation;
    }
    math::Vec3 applyInverseToDirection(const math::Vec3& d) const
    {
        return rotation.inverse().rotate(d);
    }

    /// @brief Reverse transform (B -> A).
    Transform inverse() const
    {
        const math::Quat qInv = rotation.inverse();
        return {rotation.rotate(translation) * -1.0, qInv};
    }

    /// @brief World(NED) -> aircraft body transform for the given pose.
    static Transform nedToBody(const Pose& pose)
    {
        return Transform{pose.position, pose.orientation};
    }
};

} // namespace kinematics
} // namespace core

#endif // KINEMATICS_TRANSFORM_H
