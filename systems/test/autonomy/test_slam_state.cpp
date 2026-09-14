#include <cassert>
#include <cstdint>
#include <optional>
#include <vector>

#include <autonomy/SLAM/SlamMeasurement.h>
#include <autonomy/SLAM/SlamState.h>

/// @brief Unit tests for autonomy/SlamState and SlamMeasurement.
int main()
{
    using systems::autonomy::SlamLandmark;
    using systems::autonomy::SlamMeasurement;
    using systems::autonomy::SlamState;

    /// @brief --- Default SlamState: null pose, empty map ---
    {
        SlamState s;
        assert(s.pose.position.x == 0.0);
        assert(s.pose.position.y == 0.0);
        assert(s.pose.position.z == 0.0);
        assert(s.landmarks.empty());
    }

    /// @brief --- SlamLandmark field assignment ---
    {
        SlamLandmark lm;
        lm.id = 7;
        lm.positionNed = core::math::Vec3{10.0, 20.0, -5.0};

        assert(lm.id == 7);
        assert(lm.positionNed.x == 10.0);
        assert(lm.positionNed.y == 20.0);
        assert(lm.positionNed.z == -5.0);
    }

    /// @brief --- SlamMeasurement: inherits Spherical, has optional id ---
    {
        SlamMeasurement m;
        assert(m.range == 0.0);
        assert(m.azimuth == 0.0);
        assert(m.elevation == 0.0);
        assert(!m.landmarkId.has_value());

        m.range = 100.0;
        m.azimuth = 0.2;
        m.elevation = -0.1;
        m.landmarkId = std::uint32_t{3};

        assert(m.range == 100.0);
        assert(m.landmarkId.has_value());
        assert(*m.landmarkId == 3);
    }

    return 0;
}
