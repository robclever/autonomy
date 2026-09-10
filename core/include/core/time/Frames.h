#pragma once

#include "core/math/Quat.h"
#include "core/math/Vec3.h"

namespace slam {

// Coordinate frames of the aircraft domain.
//
//   NED  — North/East/Down, the absolute reference frame of the world
//          (x = north, y = east, z = down; right-handed).
//   BODY — local frame of the aircraft (x = forward, y = right/starboard,
//          z = down).
enum class FrameId
{
    NED,
    BODY
};

// Where the aircraft is and how it is oriented in the world.
struct Pose
{
    math::Vec3 position;    // aircraft origin, expressed in NED [m]
    math::Quat orientation; // NED -> body
};

// Transform from a source frame A to a target frame B.
// A Transform is fully described by where B's origin sits in A, plus the
// A -> B rotation. Use `Transform::nedToBody` for the primary world->body map.
struct Transform
{
    math::Vec3 translation{0.0, 0.0, 0.0};   // B's origin, expressed in A
    math::Quat rotation{1.0, 0.0, 0.0, 0.0}; // A -> B

    // Point/direction from A into B.
    math::Vec3 applyToPoint(const math::Vec3& p) const { return rotation.rotate(p - translation); }
    math::Vec3 applyToDirection(const math::Vec3& d) const { return rotation.rotate(d); }

    // Point/direction from B back into A.
    math::Vec3 applyInverseToPoint(const math::Vec3& p) const {
        return rotation.inverse().rotate(p) + translation;
    }
    math::Vec3 applyInverseToDirection(const math::Vec3& d) const {
        return rotation.inverse().rotate(d);
    }

    // Reverse transform (B -> A).
    Transform inverse() const {
        const math::Quat qInv = rotation.inverse();
        return {rotation.rotate(translation) * -1.0, qInv};
    }

    // World(NED) -> aircraft body transform for the given pose.
    static Transform nedToBody(const Pose& pose) {
        return Transform{pose.position, pose.orientation};
    }
};

} // namespace slam