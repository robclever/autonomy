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
constexpr double kPi = 3.14159265358979323846;
}

// Turn scenario: aircraft flies north, turns 90 deg right, flies east.
// Verifies the pipeline handles changing viewpoints and that landmarks are
// observed from multiple angles.
int main()
{
    using namespace sim::scenarios;
    using systems::sensors::radar::RadarParams;
    using systems::sensors::radar::RadarSensor;

    // --- World: landmarks around the L-shaped path ---
    sim::World world;
    // Along the north leg
    world.addLandmark("north_1", core::math::Vec3{300.0, 100.0, -500.0}, 1.0);
    world.addLandmark("north_2", core::math::Vec3{600.0, -80.0, -500.0}, 1.0);
    // Near the corner
    world.addLandmark("corner", core::math::Vec3{800.0, 200.0, -500.0}, 1.0);
    // Along the east leg
    world.addLandmark("east_1", core::math::Vec3{1000.0, 500.0, -500.0}, 1.0);
    world.addLandmark("east_2", core::math::Vec3{1200.0, 800.0, -500.0}, 1.0);

    // --- Trajectory: north, turn right, east ---
    // Start at origin heading north. Fly 800 m north, turn 90 deg right
    // (to heading east), fly 600 m east.
    TrajectoryBuilder builder(core::math::Vec3{0.0, 0.0, -500.0}, 0.0, 0.0);
    builder.flyStraight(800.0, 50.0, 0.5);   // north leg
    builder.turnTo(kPi / 2, 0.1, 50.0, 0.5); // turn east
    builder.flyStraight(600.0, 50.0, 0.5);   // east leg
    auto traj = std::make_unique<ListTrajectory>(builder.build());
    std::cout << "turn scenario: " << traj->numSteps() << " steps\n";

    // --- Radar ---
    RadarParams params;
    params.maxRangeM = 2000.0;
    params.minRangeM = 1.0;
    params.detectionThreshold = 0.0;
    params.rangeNoiseStdDev = 0.0;
    params.azimuthNoiseStdDev = 0.0;
    params.elevationNoiseStdDev = 0.0;
    RadarSensor sensor(params);
    sim::RadarDevice device(sensor);

    // --- Sensor suite ---
    SensorSuite suite;
    suite.addDevice(std::make_unique<sim::RadarDevice>(device));

    // --- SLAM ---
    systems::autonomy::EkfSlam slam;

    // --- Scenario ---
    Scenario scenario(std::move(world), std::move(traj), std::move(suite), slam, 0.5);
    scenario.run();

    const auto& steps = scenario.steps();
    assert(!steps.empty());

    // --- Verifications ---

    // 1. The aircraft heading changed: started north (yaw~0), ended east
    //    (yaw~pi/2). We check the position delta between early and late steps.
    {
        // Early step: aircraft near start.
        const auto& early = steps[1];
        // Late step: aircraft near end.
        const auto& late = steps.back();
        // Should have moved both north and east.
        assert(late.truePose.position.x > early.truePose.position.x + 200.0);
        assert(late.truePose.position.y > early.truePose.position.y + 200.0);
        std::cout << "turn scenario: aircraft moved from (" << early.truePose.position.x << ","
                  << early.truePose.position.y << ") to (" << late.truePose.position.x << ","
                  << late.truePose.position.y << ")\n";
    }

    // 2. Landmarks were observed from multiple angles. The "corner" landmark
    //    should appear at different azimuths as the aircraft turns.
    {
        bool sawPositiveAz = false;
        bool sawNegativeAz = false;
        for (const auto& s : steps)
        {
            for (const auto& d : s.detections)
            {
                // Corner landmark is at range ~sqrt(800^2+200^2) ~= 824 m
                if (std::fabs(d.range - 824.0) < 100.0)
                {
                    if (d.azimuth > 0.1)
                        sawPositiveAz = true;
                    if (d.azimuth < -0.1)
                        sawNegativeAz = true;
                }
            }
        }
        // Before the turn the corner is to the right (+az); after turning
        // east it's behind/to the left. We should see it on both sides.
        assert(sawPositiveAz && "corner landmark should appear at positive azimuth");
        assert(sawNegativeAz && "corner landmark should appear at negative azimuth");
        std::cout << "turn scenario: corner landmark seen from multiple angles\n";
    }

    // 3. SLAM tracked pose throughout.
    for (const auto& s : steps)
    {
        assert(std::fabs(s.slamAfter.pose.position.x - s.truePose.position.x) < 1e-9);
        assert(std::fabs(s.slamAfter.pose.position.y - s.truePose.position.y) < 1e-9);
    }

    // 4. Detections were produced across the scenario (radar active throughout).
    std::size_t detectionSteps = 0;
    for (const auto& s : steps)
    {
        if (!s.detections.empty())
            ++detectionSteps;
    }
    assert(detectionSteps > 5);
    std::cout << "turn scenario: " << detectionSteps << " steps had detections\n";

    std::cout << "turn scenario: PASSED\n";
    return 0;
}
