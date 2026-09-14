#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

#include <autonomy/SLAM/EkfSlam.h>
#include <environment/RadarDevice.h>
#include <scenarios/Scenario.h>
#include <scenarios/SensorSuite.h>
#include <scenarios/TrajectoryBuilder.h>
#include <sensors/radar/RadarSensor.h>

namespace
{
constexpr double kTol = 1e-6;
void expectNear(double a, double b, double tol = 1e-6) { assert(std::fabs(a - b) <= tol); }
} // namespace

/// @brief Basic scenario: aircraft flies straight north past landmarks.
/// Verifies the full pipeline runs step-by-step and produces sensible output.
int main()
{
    using namespace sim::scenarios;
    using systems::sensors::radar::RadarParams;
    using systems::sensors::radar::RadarSensor;

    /// @brief --- World: landmarks spread out north and to the sides ---
    sim::World world;
    world.addLandmark("tower_near", core::math::Vec3{200.0, 50.0, -500.0}, 1.0);
    world.addLandmark("tower_mid", core::math::Vec3{600.0, -30.0, -500.0}, 1.0);
    world.addLandmark("tower_far", core::math::Vec3{1000.0, 80.0, -500.0}, 1.0);

    /// @brief --- Trajectory: start south of landmarks, fly north ---
    /// Start at (0,0,-500), heading 0 (north), fly 1200 m at 50 m/s.
    TrajectoryBuilder builder(core::math::Vec3{0.0, 0.0, -500.0}, 0.0, 0.0);
    builder.flyStraight(1200.0, 50.0, 0.5);
    auto traj = std::make_unique<ListTrajectory>(builder.build());
    std::cout << "basic scenario: " << traj->numSteps() << " steps\n";

    /// @brief --- Radar: wide field, low detection threshold so we see everything ---
    RadarParams params;
    params.maxRangeM = 2000.0;
    params.minRangeM = 1.0;
    params.detectionThreshold = 0.0;
    params.rangeNoiseStdDev = 0.0; // no noise for deterministic test
    params.azimuthNoiseStdDev = 0.0;
    params.elevationNoiseStdDev = 0.0;
    RadarSensor sensor(params);
    sim::RadarDevice device(sensor);

    /// @brief --- Sensor suite (can hold multiple sensors) ---
    SensorSuite suite;
    suite.addDevice(std::make_unique<sim::RadarDevice>(device));

    /// @brief --- SLAM ---
    systems::autonomy::EkfSlam slam;

    /// @brief --- Scenario ---
    Scenario scenario(std::move(world), std::move(traj), std::move(suite), slam, 0.5);
    scenario.run();

    const auto& steps = scenario.steps();
    assert(!steps.empty());
    std::cout << "basic scenario: " << steps.size() << " steps recorded\n";

    /// @brief --- Verifications ---

    /// @brief 1. SLAM tracked the aircraft pose at every step.
    for (const auto& s : steps)
    {
        expectNear(s.slamAfter.pose.position.x, s.truePose.position.x, 1e-9);
        expectNear(s.slamAfter.pose.position.y, s.truePose.position.y, 1e-9);
        expectNear(s.slamAfter.pose.position.z, s.truePose.position.z, 1e-9);
    }

    /// @brief 2. At the first step, the near landmark (200 m ahead) should be in view.
    {
        bool found = false;
        for (const auto& d : steps[0].detections)
        {
            if (std::fabs(d.range - 200.0) < 50.0)
                found = true;
        }
        assert(found && "near landmark should be detected at step 0");
    }

    /// @brief 3. Detections are produced at multiple steps (radar is working throughout).
    std::size_t stepsWithDetections = 0;
    for (const auto& s : steps)
    {
        if (!s.detections.empty())
            ++stepsWithDetections;
    }
    assert(stepsWithDetections > 5 && "radar should produce detections across many steps");
    std::cout << "basic scenario: " << stepsWithDetections << " steps had detections\n";

    /// @brief 4. Measurements are produced and passed to SLAM.
    {
        bool foundMeas = false;
        for (const auto& s : steps)
        {
            if (!s.measurements.empty())
                foundMeas = true;
        }
        assert(foundMeas && "measurements should be produced at some steps");
    }

    /// @brief 5. The aircraft advanced north over the scenario.
    const double startX = steps.front().truePose.position.x;
    const double endX = steps.back().truePose.position.x;
    assert(endX > startX + 500.0 && "aircraft should have flown north");
    std::cout << "basic scenario: aircraft flew from x=" << startX << " to x=" << endX << "\n";

    std::cout << "basic scenario: PASSED\n";
    return 0;
}
