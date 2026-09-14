#include <cassert>
#include <cmath>
#include <vector>

#include <environment/RadarDevice.h>
#include <sensors/radar/RadarSensor.h>

namespace
{
constexpr double kTol = 1e-9;
void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
} // namespace

// Unit tests for sim environment/RadarDevice.
int main()
{
    using systems::sensors::radar::RadarParams;
    using systems::sensors::radar::RadarSensor;

    // --- Device produces echoes from a world with one in-range landmark ---
    {
        sim::World world;
        world.addLandmark("tower", core::math::Vec3{500.0, 0.0, -100.0}, 1.0);

        RadarParams params;
        params.minRangeM = 1.0;
        params.maxRangeM = 1000.0;
        params.detectionThreshold = 0.0; // accept all in range
        RadarSensor sensor(params);

        sim::RadarDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, -100.0},
                                              core::math::Quat::identity()};

        const auto echoes = device.senseEchoes(world, observer);
        assert(!echoes.empty());
        // Range should be near 500 m (with noise).
        assert(std::fabs(echoes[0].range - 500.0) < 50.0);
    }

    // --- Device with high detection threshold drops weak targets ---
    {
        sim::World world;
        // Weak reflectivity at long range -> low detectibility.
        world.addLandmark("far_weak", core::math::Vec3{900.0, 0.0, -100.0}, 0.001);

        RadarParams params;
        params.minRangeM = 1.0;
        params.maxRangeM = 5000.0;
        params.detectionThreshold = 0.9; // very high bar
        RadarSensor sensor(params);

        sim::RadarDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, -100.0},
                                              core::math::Quat::identity()};

        const auto echoes = device.senseEchoes(world, observer);
        assert(echoes.empty()); // nothing passes the threshold
    }

    // --- Empty world produces no echoes ---
    {
        sim::World world;
        RadarSensor sensor;
        sim::RadarDevice device(sensor);

        const core::kinematics::Pose observer{core::math::Vec3{0.0, 0.0, 0.0},
                                              core::math::Quat::identity()};

        const auto echoes = device.senseEchoes(world, observer);
        assert(echoes.empty());
    }

    return 0;
}
