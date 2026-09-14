#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

#include <autonomy/SLAM/EkfSlam.h>
#include <autonomy/SLAM/GraphSlam.h>
#include <environment/RadarDevice.h>
#include <scenarios/Scenario.h>
#include <scenarios/SensorSuite.h>
#include <scenarios/TrajectoryBuilder.h>
#include <sensors/radar/RadarSensor.h>

namespace
{
/// @brief Run a full SLAM scenario with ground-truth association and verify the
/// algorithm's state evolves correctly. Templated on the SLAM type so we
/// exercise both EkfSlam and GraphSlam through the same pipeline.
template <typename SlamAlgo> void runSlamScenario(const char* name)
{
    using namespace sim::scenarios;
    using systems::sensors::radar::RadarParams;
    using systems::sensors::radar::RadarSensor;

    /// @brief --- World: 5 landmarks at known positions ---
    sim::World world;
    world.addLandmark("alpha", core::math::Vec3{250.0, 60.0, -500.0}, 1.0);
    world.addLandmark("bravo", core::math::Vec3{500.0, -40.0, -500.0}, 1.0);
    world.addLandmark("charlie", core::math::Vec3{750.0, 70.0, -500.0}, 1.0);
    world.addLandmark("delta", core::math::Vec3{1000.0, -60.0, -500.0}, 1.0);
    world.addLandmark("echo", core::math::Vec3{1250.0, 30.0, -500.0}, 1.0);

    /// @brief --- Trajectory: fly north past all landmarks ---
    TrajectoryBuilder builder(core::math::Vec3{0.0, 0.0, -500.0}, 0.0, 0.0);
    builder.flyStraight(1500.0, 50.0, 0.5);
    auto traj = std::make_unique<ListTrajectory>(builder.build());

    /// @brief --- Radar: deterministic (no noise) so echoes match ground truth ---
    RadarParams params;
    params.maxRangeM = 2000.0;
    params.minRangeM = 1.0;
    params.detectionThreshold = 0.0;
    params.rangeNoiseStdDev = 0.0;
    params.azimuthNoiseStdDev = 0.0;
    params.elevationNoiseStdDev = 0.0;
    RadarSensor sensor(params);
    sim::RadarDevice device(sensor);

    /// @brief --- Sensor suite ---
    SensorSuite suite;
    suite.addDevice(std::make_unique<sim::RadarDevice>(device));

    /// @brief --- SLAM with ground-truth association ---
    SlamAlgo slam;
    Scenario scenario(std::move(world), std::move(traj), std::move(suite), slam, 0.5);
    scenario.setUseAssociation(true);
    scenario.run();

    const auto& steps = scenario.steps();
    assert(!steps.empty());

    std::cout << name << ": " << steps.size() << " steps\n";

    /// @brief --- Verifications ---

    /// @brief 1. SLAM tracked the aircraft pose at every step.
    for (const auto& s : steps)
    {
        assert(std::fabs(s.slamAfter.pose.position.x - s.truePose.position.x) < 1e-9);
        assert(std::fabs(s.slamAfter.pose.position.y - s.truePose.position.y) < 1e-9);
        assert(std::fabs(s.slamAfter.pose.position.z - s.truePose.position.z) < 1e-9);
    }

    /// @brief 2. Landmarks were added to the SLAM map. With ground-truth association
    ///    and all landmarks in range, the final map should contain all 5.
    const auto finalState = slam.state();
    std::cout << name << ": final map has " << finalState.landmarks.size() << " landmarks\n";
    assert(finalState.landmarks.size() == 5 && "all 5 landmarks should be in the SLAM map");

    /// @brief 3. No duplicate landmark IDs in the map.
    for (std::size_t i = 0; i < finalState.landmarks.size(); ++i)
    {
        for (std::size_t j = i + 1; j < finalState.landmarks.size(); ++j)
        {
            assert(finalState.landmarks[i].id != finalState.landmarks[j].id &&
                   "no duplicate landmark IDs");
        }
    }

    /// @brief 4. Landmarks were added incrementally: the map grew over time (not all
    ///    at once at the first step).
    std::size_t maxMapSize = 0;
    std::size_t firstFullStep = 0;
    for (std::size_t i = 0; i < steps.size(); ++i)
    {
        const auto size = steps[i].slamAfter.landmarks.size();
        if (size > maxMapSize)
            maxMapSize = size;
        if (size == 5 && firstFullStep == 0)
            firstFullStep = i;
    }
    assert(maxMapSize == 5);
    assert(firstFullStep > 0 && "map should grow over time, not be full at step 0");
    std::cout << name << ": map full at step " << firstFullStep << " of " << steps.size() - 1
              << "\n";

    /// @brief 5. At the step where a landmark first appears, it was actually observed
    ///    (had an echo) at that step or earlier. We verify the first step that
    ///    reports 3+ landmarks had echoes at that step.
    for (const auto& s : steps)
    {
        if (s.slamAfter.landmarks.size() >= 3)
        {
            assert(!s.detections.empty() && "step with 3+ landmarks should have had detections");
            break;
        }
    }

    /// @brief 6. Measurements carried landmark IDs (association worked).
    std::size_t associatedCount = 0;
    for (const auto& s : steps)
    {
        for (const auto& m : s.measurements)
        {
            if (m.landmarkId.has_value())
                ++associatedCount;
        }
    }
    assert(associatedCount > 0 && "ground-truth association should produce IDs");
    std::cout << name << ": " << associatedCount << " associated measurements\n";

    /// @brief 7. Reset works: after reset, the map is empty again.
    slam.reset();
    assert(slam.state().landmarks.empty() && "reset should clear the map");

    std::cout << name << ": PASSED\n";
}
} // namespace

/// @brief Full SLAM pipeline scenario with ground-truth data association.
/// Exercises both EKF and Graph SLAM through the same pipeline.
int main()
{
    runSlamScenario<systems::autonomy::EkfSlam>("slam_ekf");
    runSlamScenario<systems::autonomy::GraphSlam>("slam_graph");
    return 0;
}
