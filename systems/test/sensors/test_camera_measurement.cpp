#include <cassert>

#include <sensors/camera/CameraMeasurement.h>

// Unit tests for sensors/camera/CameraMeasurement.
int main()
{
    // --- Default construction ---
    {
        systems::sensors::camera::CameraMeasurement m;
        assert(m.range == 0.0);
        assert(m.azimuth == 0.0);
        assert(m.elevation == 0.0);
        assert(m.contrast == 0.0);
        assert(m.brightness == 0.0);
        assert(m.detectionConfidence == 0.0);
    }

    // --- Field assignment ---
    {
        systems::sensors::camera::CameraMeasurement m;
        m.range = 100.0;
        m.azimuth = 0.05;
        m.elevation = -0.02;
        m.contrast = 0.8;
        m.brightness = 0.9;
        m.detectionConfidence = 0.75;

        assert(m.range == 100.0);
        assert(m.azimuth == 0.05);
        assert(m.elevation == -0.02);
        assert(m.contrast == 0.8);
        assert(m.brightness == 0.9);
        assert(m.detectionConfidence == 0.75);
    }

    return 0;
}
