#pragma once

namespace core {
namespace kinematics {

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

} // namespace kinematics
} // namespace core
