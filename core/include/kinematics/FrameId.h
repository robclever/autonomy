#ifndef KINEMATICS_FRAMEID_H
#define KINEMATICS_FRAMEID_H

namespace core
{
namespace kinematics
{

/// @brief Coordinate frames of the aircraft domain.
///
///   NED  — North/East/Down, the absolute reference frame of the world
///          (x = north, y = east, z = down; right-handed).
///   BODY — local frame of the aircraft (x = forward, y = right/starboard,
///          z = down).
enum class FrameId
{
    NED,
    BODY
};

} // namespace kinematics
} // namespace core

#endif // KINEMATICS_FRAMEID_H
