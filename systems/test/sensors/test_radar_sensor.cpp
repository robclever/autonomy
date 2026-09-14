#include <cassert>
#include <cmath>
#include <vector>

#include <sensors/radar/RadarSensor.h>

namespace
{
constexpr double kTol = 1e-9;

void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }

// Helper to build a simple observation.
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

// Unit tests for sensors/radar/RadarSensor.
int main()
{
    using systems::sensors::radar::RadarSensor;

    // --- No observations -> no echoes ---
    {
        RadarSensor sensor;
        const auto echoes = sensor.process({});
        assert(echoes.empty());
    }

    // --- Range gating: target beyond maxRange is dropped ---
    {
        systems::sensors::radar::RadarParams params;
        params.maxRangeM = 500.0;
        params.minRangeM = 1.0;
        params.detectionThreshold = 0.0; // accept everything in range
        RadarSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(1000.0, 0.0, 0.0, 1e6)); // beyond max, strong signal
        obs.push_back(makeObs(100.0, 0.0, 0.0, 1e6));  // in range

        const auto echoes = sensor.process(obs);
        assert(echoes.size() == 1);
        expectNear(echoes[0].range, 100.0, 50.0); // noisy, but near 100
    }

    // --- Range gating: target closer than minRange is dropped ---
    {
        systems::sensors::radar::RadarParams params;
        params.minRangeM = 5.0;
        params.maxRangeM = 1000.0;
        params.detectionThreshold = 0.0;
        RadarSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(0.5, 0.0, 0.0, 1e6));  // too close
        obs.push_back(makeObs(50.0, 0.0, 0.0, 1e6)); // ok

        const auto echoes = sensor.process(obs);
        assert(echoes.size() == 1);
        expectNear(echoes[0].range, 50.0, 50.0);
    }

    // --- Detection threshold: weak signal dropped, strong signal kept ---
    {
        systems::sensors::radar::RadarParams params;
        params.minRangeM = 1.0;
        params.maxRangeM = 10000.0;
        params.detectionThreshold = 0.5;
        RadarSensor sensor(params);

        std::vector<sim::Observation> obs;
        // signalStrength=0.01 at range=100 -> detectibility = 0.01/10000 = 1e-6 (dropped)
        obs.push_back(makeObs(100.0, 0.0, 0.0, 0.01));
        // signalStrength=1e6 at range=100 -> detectibility = min(1, 1e6/1e4)=1.0 (kept)
        obs.push_back(makeObs(100.0, 0.0, 0.0, 1e6));

        const auto echoes = sensor.process(obs);
        assert(echoes.size() == 1);
        expectNear(echoes[0].likelihoodOfDetect, 1.0, kTol);
    }

    // --- Echo carries signature and detection fields ---
    {
        systems::sensors::radar::RadarParams params;
        params.detectionThreshold = 0.0;
        RadarSensor sensor(params);

        std::vector<sim::Observation> obs;
        obs.push_back(makeObs(200.0, 0.3, -0.1, 5000.0));

        const auto echoes = sensor.process(obs);
        assert(echoes.size() == 1);
        expectNear(echoes[0].signature, 5000.0, kTol);
        assert(echoes[0].detectibility > 0.0);
        expectNear(echoes[0].likelihoodOfDetect, echoes[0].detectibility, kTol);
    }

    return 0;
}
