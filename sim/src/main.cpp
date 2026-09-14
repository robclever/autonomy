#include <iomanip>
#include <iostream>

#include "environment/World.h"
#include "math/Quat.h"
#include "math/Vec3.h"
#include "time/Clock.h"

// Temporary bootstrap: proves the build and the first vertical slice
// (world model -> structured observations from an aircraft pose). Sensor,
// fusion, autonomy, and platform stages are still stubs; see .README.
int main()
{
    using core::math::Vec3;

    core::time::Clock clock(10'000'000); // 10 ms steps

    sim::World world;
    world.addLandmark("water_tower", Vec3{1200.0, 300.0, -50.0});
    world.addLandmark("power_line_pole", Vec3{-40.0, 90.0, -15.0});
    world.addLandmark("runway_end", Vec3{5000.0, 0.0, 0.0});

    const core::kinematics::Pose observer{Vec3{0.0, 0.0, -400.0},
                                          core::math::Quat::fromYawPitchRoll(0.8, 0.12, -0.05)};

    std::cout << "slam_sim: aircraft autonomy & SLAM learning sandbox\n";
    std::cout << "clock step        : " << clock.step_ns() << " ns\n";
    std::cout << "world landmarks   : " << world.landmarks().size() << "\n";
    std::cout << "observer position : NED(" << observer.position.x << ", " << observer.position.y
              << ", " << observer.position.z << ") m\n";

    const auto obs = world.observe(observer, {1000.0, 1.0});
    std::cout << "structured observations within 1000 m:\n";
    std::cout << std::fixed << std::setprecision(2);
    for (const auto& o : obs)
    {
        std::cout << "  id " << std::setw(2) << o.id << "  range " << std::setw(7) << o.rangeM
                  << " m  az " << std::setw(7) << o.azimuthRad << " rad  el " << std::setw(7)
                  << o.elevationRad << " rad  strength " << o.signalStrength << "\n";
    }

    std::cout << "sensors, fusion, autonomy, platform: stubs\n";
    return 0;
}