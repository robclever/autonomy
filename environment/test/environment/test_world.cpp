#include <cassert>
#include <cmath>
#include <optional>
#include <vector>

#include <Landmark.h>
#include <environment/World.h>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }
void expectNear(const core::math::Vec3& a, const core::math::Vec3& b, double tol = kTol)
{
    expectNear(a.x, b.x, tol);
    expectNear(a.y, b.y, tol);
    expectNear(a.z, b.z, tol);
}
} // namespace

// Unit tests for environment/World.
int main()
{
    using core::math::Vec3;
    using sim::Landmark;

    // --- Empty world produces no observations ---
    {
        sim::World world;
        assert(world.landmarks().empty());

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);
        assert(obs.empty());
    }

    // --- addLandmark assigns sequential IDs starting at 1 ---
    {
        sim::World world;
        const auto id1 = world.addLandmark("first", Vec3{10.0, 0.0, 0.0});
        const auto id2 = world.addLandmark("second", Vec3{20.0, 0.0, 0.0});
        const auto id3 = world.addLandmark("third", Vec3{30.0, 0.0, 0.0}, 0.5);

        assert(id1 == 1);
        assert(id2 == 2);
        assert(id3 == 3);
        assert(world.landmarks().size() == 3);
    }

    // --- addLandmark stores landmark fields correctly ---
    {
        sim::World world;
        const auto id = world.addLandmark("tower", Vec3{100.0, 200.0, -50.0}, 0.8);

        const auto lm = world.landmark(id);
        assert(lm.has_value());
        assert(lm->id == id);
        assert(lm->name == "tower");
        expectNear(lm->positionNed, Vec3{100.0, 200.0, -50.0});
        assert(lm->reflectivity == 0.8);
    }

    // --- landmark() returns nullopt for unknown ID ---
    {
        sim::World world;
        world.addLandmark("only", Vec3{1.0, 2.0, 3.0});

        assert(!world.landmark(0).has_value());
        assert(!world.landmark(99).has_value());
    }

    // --- observe: single landmark directly ahead at known range ---
    {
        sim::World world;
        world.addLandmark("ahead", Vec3{500.0, 0.0, -100.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        expectNear(obs[0].rangeM, 500.0);
        expectNear(obs[0].azimuthRad, 0.0);   // straight ahead
        expectNear(obs[0].elevationRad, 0.0); // same altitude
        expectNear(obs[0].positionNed, Vec3{500.0, 0.0, -100.0});
    }

    // --- observe: landmark to the right has positive azimuth ---
    {
        sim::World world;
        world.addLandmark("right", Vec3{0.0, 300.0, -100.0});

        const core::kinematics::Pose observer{Vec3{0.0, 0.0, -100.0}, core::math::Quat::identity()};
        const auto obs = world.observe(observer);

        assert(obs.size() == 1);
        expectNear(obs[0].rangeM, 300.0);
        expectNear(obs[0].azimuthRad, 0.5 * kPi); // 90 deg right
        expectNear(obs[0].elevationRad, 0.0);
    }

    return 0;
}
