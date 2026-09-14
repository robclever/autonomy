#ifndef AUTONOMY_SLAM_SLAMMEASUREMENT_H
#define AUTONOMY_SLAM_SLAMMEASUREMENT_H

#include <cstdint>
#include <optional>

#include <measurement/Spherical.h>

namespace systems
{
namespace autonomy
{

// A single landmark measurement as consumed by SLAM. Extends the base
// Spherical measurement with an optional landmark identifier. When the id is
// known (data association already resolved) it is set; otherwise the SLAM
// algorithm must associate it itself.
struct SlamMeasurement : public core::measurement::Spherical
{
    std::optional<std::uint32_t> landmarkId; // associated landmark, if known
};

} // namespace autonomy
} // namespace systems

#endif // AUTONOMY_SLAM_SLAMMEASUREMENT_H
