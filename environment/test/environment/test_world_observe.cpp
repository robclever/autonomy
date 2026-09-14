#include <cassert>
#include <cmath>
#include <vector>

#include <Landmark.h>
#include <environment/World.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
} // namespace

// Additional observe() tests for environment/World.
int main()
{
    using core::math::Vec3;

    // --- observe: landmark behind has azimuth == +-pi ---
    {
        sim::World world;
        world.addLandmark("behind", Vec3{-400.0, 0.0, -100.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        expectNear(obs[0].rangeM, 400.0);
        expectNear(std::fabs(obs[0].azimuthRad), kPi);
    }

    // --- observe: maxRangeM filtering ---
    {
        sim::World world;
        world.addLandmark("near", Vec3{50.0, 0.0, -100.0});
        world.addLandmark("far", Vec3{5000.0, 0.0, -100.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const sim::ObservationParams params{1000.0, 1.0};
        const auto obs = world.observe(observer, params);

        assert(obs.size() == 1);
        assert(obs[0].id == 1); // only the near landmark
    }

    // --- observe: minRangeM filtering ---
    {
        sim::World world;
        world.addLandmark("too_close", Vec3{0.5, 0.0, -100.0});
        world.addLandmark("ok", Vec3{50.0, 0.0, -100.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const sim::ObservationParams params{1000.0, 1.0};
        const auto obs = world.observe(observer, params);

        assert(obs.size() == 1);
        assert(obs[0].id == 2); // only the ok landmark
    }

    // --- observe: signal strength falls off with 1/r^2 ---
    {
        sim::World world;
        world.addLandmark("target", Vec3{100.0, 0.0, -100.0}, 2.0);

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        // signalStrength = reflectivity / range^2 = 2.0 / 10000.0
        expectNear(obs[0].signalStrength, 2.0 / 10000.0);
    }

    // --- observe: directionBody is a unit vector ---
    {
        sim::World world;
        world.addLandmark("target", Vec3{123.0, 456.0, -789.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        expectNear(obs[0].directionBody.norm(), 1.0, 1e-9);
    }

    // --- observe: with a yaw rotation, azimuth shifts accordingly ---
    {
        sim::World world;
        world.addLandmark("north", Vec3{100.0, 0.0, -100.0});

        // Aircraft yaws +90 deg about Down: the NED North axis maps to body
        // Right, so the landmark to the north appears at +90 deg azimuth.
        const core::kinematics::Pose observer{
            Vec3{0.0, 0.0, -100.0}, core::math::Quat::fromYawPitchRoll(0.5 * kPi, 0.0, 0.0)};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        expectNear(obs[0].azimuthRad, 0.5 * kPi, 1e-9);
    }

    return 0;
}
