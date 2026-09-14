#include <cassert>
#include <cmath>
#include <vector>

#include <environment/CameraDevice.h>
#include <sensors/camera/CameraSensor.h>

namespace
{
constexpr double kTol = 1e-9;
void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
} // namespace

/// @brief Unit tests for sim environment/CameraDevice.
int main()
{
    using systems::sensors::camera::CameraParams;
    using systems::sensors::camera::CameraSensor;

    /// @brief --- Device produces detections from a world with one in-range landmark ---
    {
        sim::World world;
        world.addLandmark("tower", core::math::Vec3{200.0, 0.0, -100.0}, 1.0);

        CameraParams params;
        params.minRangeM = 1.0;
        params.maxRangeM = 500.0;
        params.maxAzimuthFOVRad = M_PI / 2.0;
        params.maxElevationFOVRad = M_PI / 2.0;
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);

        sim::CameraDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, -100.0},
                                              core::math::Quat::identity()};

        const auto dets = device.sense(world, observer);
        assert(!dets.empty());
        /// @brief Range should be near 200 m.
        assert(std::fabs(dets[0].range - 200.0) < 50.0);
        /// @brief Detection has confidence.
        assert(dets[0].confidence > 0.0);
    }

    /// @brief --- Device produces generic Detections (not camera-specific) ---
    {
        sim::World world;
        world.addLandmark("close", core::math::Vec3{50.0, 10.0, -100.0}, 1.0);

        CameraParams params;
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);
        sim::CameraDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, -100.0},
                                              core::math::Quat::identity()};

        const auto dets = device.sense(world, observer);
        assert(!dets.empty());
        /// @brief Detections are generic: have range/azimuth/elevation/confidence
        expectNear(dets[0].azimuth, std::atan2(10.0, 50.0), 0.01);
    }

    /// @brief --- Empty world produces no detections ---
    {
        sim::World world;
        CameraSensor sensor;
        sim::CameraDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, 0.0},
                                              core::math::Quat::identity()};

        const auto dets = device.sense(world, observer);
        assert(dets.empty());
    }

    return 0;
}
