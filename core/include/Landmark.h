#ifndef LANDMARK_H
#define LANDMARK_H

#include <cstdint>
#include <string>

#include <math/Vec3.h>

namespace sim
{

// A fixed, observable thing in the world (tower, hangar, terrain feature...).
struct Landmark
{
    std::uint32_t id{0};
    std::string name;
    core::math::Vec3 positionNed; // [m], NED coordinates
    double reflectivity{1.0};     // placeholder: how strongly it "returns" energy
};

} // namespace sim

#endif // LANDMARK_H
