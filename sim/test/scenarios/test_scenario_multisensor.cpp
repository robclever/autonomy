#include <cassert>
#include <cmath>
#include <iostream>
#include <memory>

#include <autonomy/SLAM/EkfSlam.h>
#include <environment/CameraDevice.h>
#include <environment/RadarDevice.h>
#include <scenarios/Scenario.h>
#include <scenarios/SensorSuite.h>
#include <scenarios/TrajectoryBuilder.h>
#include <sensors/camera/CameraSensor.h>
#include <sensors/radar/RadarSensor.h>

/// @brief Multi-sensor scenario: aircraft flies with BOTH radar and camera.
/// Verifies the SensorSuite merges detections from different sensor types and
/// that SLAM receives measurements from both.
int main()
{
    using namespace sim::scenarios;
    using systems::sensors::camera::CameraParams;
    using systems::sensors::camera::CameraSensor;
    using systems::sensors::radar::RadarParams;
    using systems::sensors::radar::RadarSensor;

    /// @brief --- World: landmarks at various ranges and angles ---
    sim::World world;
    world.addLandmark("near", core::math::Vec3{150.0, 30.0, -500.0}, 1.0);
    world.addLandmark("mid", core::math::Vec3{400.0, -50.0, -500.0}, 1.0);
    world.addLandmark("far", core::math::Vec3{800.0, 60.0, -500.0}, 1.0);

    /// @brief --- Trajectory: fly north past the landmarks ---
    TrajectoryBuilder builder(core::math::Vec3{0.0, 0.0, -500.0}, 0.0, 0.0);
    builder.flyStraight(1000.0, 50.0, 0.5);
    auto traj = std::make_unique<ListTrajectory>(builder.build());

    /// @brief --- Radar: long range, wide FOV ---
    RadarParams radarParams;
    radarParams.maxRangeM = 2000.0;
    radarParams.minRangeM = 1.0;
    radarParams.detectionThreshold = 0.0;
    radarParams.rangeNoiseStdDev = 0.0;
    radarParams.azimuthNoiseStdDev = 0.0;
    radarParams.elevationNoiseStdDev = 0.0;
    RadarSensor radarSensor(radarParams);
    sim::RadarDevice radarDevice(radarSensor);

    /// @brief --- Camera: shorter range, narrower FOV ---
    CameraParams cameraParams;
    cameraParams.maxRangeM = 600.0;
    cameraParams.minRangeM = 1.0;
    cameraParams.maxAzimuthFOVRad = M_PI / 3.0; // +/- 60 deg
    cameraParams.maxElevationFOVRad = M_PI / 4.0;
    cameraParams.detectionThreshold = 0.0;
    cameraParams.rangeNoiseStdDev = 0.0;
    cameraParams.azimuthNoiseStdDev = 0.0;
    cameraParams.elevationNoiseStdDev = 0.0;
    CameraSensor cameraSensor(cameraParams);
    sim::CameraDevice cameraDevice(cameraSensor);

    /// @brief --- Sensor suite: both sensors on the ownship ---
    SensorSuite suite;
    suite.addDevice(std::make_unique<sim::RadarDevice>(radarDevice));
    suite.addDevice(std::make_unique<sim::CameraDevice>(cameraDevice));
    assert(suite.deviceCount() == 2);
    std::cout << "multisensor: suite has " << suite.deviceCount() << " devices\n";

    /// @brief --- SLAM with ground-truth association ---
    systems::autonomy::EkfSlam slam;
    Scenario scenario(std::move(world), std::move(traj), std::move(suite), slam, 0.5);
    scenario.setUseAssociation(true);
    scenario.run();

    const auto& steps = scenario.steps();
    assert(!steps.empty());
    std::cout << "multisensor: " << steps.size() << " steps\n";

    /// @brief --- Verifications ---

    /// @brief 1. Detections come from multiple sensors. At early steps, both radar
    ///    and camera should detect the near landmark.
    {
        std::size_t totalDetections = 0;
        for (const auto& s : steps)
            totalDetections += s.detections.size();
        assert(totalDetections > 0);
        std::cout << "multisensor: " << totalDetections << " total detections\n";
    }

    /// @brief 2. Radar sees farther than camera: at some step, the "far" landmark
    ///    (800 m) should appear in detections (from radar) even though the
    ///    camera can't see it (maxRange 600 m).
    {
        bool sawFarLandmark = false;
        for (const auto& s : steps)
        {
            for (const auto& d : s.detections)
            {
                /// @brief "far" landmark at ~800 m range
                if (std::fabs(d.range - 800.0) < 50.0)
                    sawFarLandmark = true;
            }
        }
        assert(sawFarLandmark && "radar should detect the far landmark beyond camera range");
        std::cout << "multisensor: radar detected far landmark beyond camera range\n";
    }

    /// @brief 3. SLAM tracked pose and built a map from multi-sensor input.
    {
        const auto finalState = slam.state();
        /// @brief All 3 landmarks should be in the map (radar sees all of them).
        assert(finalState.landmarks.size() == 3);
        std::cout << "multisensor: final map has " << finalState.landmarks.size() << " landmarks\n";
    }

    /// @brief 4. Measurements carried landmark IDs (association worked across sensors).
    {
        std::size_t associatedCount = 0;
        for (const auto& s : steps)
        {
            for (const auto& m : s.measurements)
            {
                if (m.landmarkId.has_value())
                    ++associatedCount;
            }
        }
        assert(associatedCount > 0);
        std::cout << "multisensor: " << associatedCount << " associated measurements\n";
    }

    /// @brief 5. SLAM pose tracked the aircraft at every step.
    for (const auto& s : steps)
    {
        assert(std::fabs(s.slamAfter.pose.position.x - s.truePose.position.x) < 1e-9);
        assert(std::fabs(s.slamAfter.pose.position.y - s.truePose.position.y) < 1e-9);
    }

    std::cout << "multisensor scenario: PASSED\n";
    return 0;
}
