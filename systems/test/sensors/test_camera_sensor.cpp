#include <cassert>
#include <cmath>
#include <vector>

#include <sensors/camera/CameraSensor.h>

namespace
{
sim::Observation makeObs(double range, double az, double el, double signal)
{
    sim::Observation o;
    o.rangeM = range;
    o.azimuthRad = az;
    o.elevationRad = el;
    o.signalStrength = signal;
    return o;
}
} // namespace

// Unit tests for sensors/camera/CameraSensor.
int main()
{
    using systems::sensors::camera::CameraSensor;

    // --- No observations -> no detections ---
    {
        CameraSensor sensor;
        const auto dets = sensor.process({});
        assert(dets.empty());
    }

    // --- Range gating ---
    {
        systems::sensors::camera::CameraParams params;
        params.maxRangeM = 200.0;
        params.minRangeM = 5.0;
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(500.0, 0.0, 0.0, 1e6)); // beyond max
        obs.push_back(makeObs(50.0, 0.0, 0.0, 1e6));  // in range

        const auto dets = sensor.process(obs);
        assert(dets.size() == 1);
    }

    // --- FOV gating: target outside azimuth FOV is dropped ---
    {
        systems::sensors::camera::CameraParams params;
        params.maxRangeM = 2000.0;
        params.minRangeM = 1.0;
        params.maxAzimuthFOVRad = M_PI / 6.0; // +/- 30 deg
        params.maxElevationFOVRad = M_PI / 6.0;
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(100.0, M_PI / 2.0, 0.0, 1e6)); // 90 deg az, outside FOV
        obs.push_back(makeObs(100.0, 0.1, 0.0, 1e6));        // inside FOV

        const auto dets = sensor.process(obs);
        assert(dets.size() == 1);
        assert(std::fabs(dets[0].azimuth - 0.1) < 0.01);
    }

    // --- FOV gating: target outside elevation FOV is dropped ---
    {
        systems::sensors::camera::CameraParams params;
        params.maxRangeM = 2000.0;
        params.minRangeM = 1.0;
        params.maxAzimuthFOVRad = M_PI / 2.0;
        params.maxElevationFOVRad = M_PI / 12.0; // +/- 15 deg
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(100.0, 0.0, M_PI / 4.0, 1e6)); // 45 deg el, outside FOV
        obs.push_back(makeObs(100.0, 0.0, 0.05, 1e6));       // inside FOV

        const auto dets = sensor.process(obs);
        assert(dets.size() == 1);
        assert(std::fabs(dets[0].elevation - 0.05) < 0.01);
    }

    // --- Detection threshold ---
    {
        systems::sensors::camera::CameraParams params;
        params.maxRangeM = 2000.0;
        params.minRangeM = 1.0;
        params.maxAzimuthFOVRad = M_PI;
        params.maxElevationFOVRad = M_PI;
        params.detectionThreshold = 0.5;
        CameraSensor sensor(params);

        std::vector<sim::Observation> obs;
        // Weak signal at long range -> low confidence (dropped)
        obs.push_back(makeObs(200.0, 0.0, 0.0, 0.01));
        // Strong signal -> high confidence (kept)
        obs.push_back(makeObs(50.0, 0.0, 0.0, 1e6));

        const auto dets = sensor.process(obs);
        assert(dets.size() == 1);
        assert(dets[0].detectionConfidence > 0.5);
    }

    // --- Detection carries camera-specific fields ---
    {
        systems::sensors::camera::CameraParams params;
        params.detectionThreshold = 0.0;
        CameraSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(80.0, 0.1, -0.05, 5000.0));

        const auto dets = sensor.process(obs);
        assert(dets.size() == 1);
        assert(dets[0].contrast > 0.0);
        assert(dets[0].brightness > 0.0);
        assert(dets[0].detectionConfidence > 0.0);
    }

    return 0;
}
