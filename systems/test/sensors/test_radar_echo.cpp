#include <cassert>

#include <sensors/radar/RadarEcho.h>

/// @brief Unit tests for sensors/radar/RadarEcho.
int main()
{
    /// @brief --- Default construction: zeroed spherical fields + zero radar params ---
    {
        systems::sensors::radar::RadarEcho echo;
        assert(echo.range == 0.0);
        assert(echo.azimuth == 0.0);
        assert(echo.elevation == 0.0);
        assert(echo.signature == 0.0);
        assert(echo.detectibility == 0.0);
        assert(echo.likelihoodOfDetect == 0.0);
    }

    /// @brief --- Field assignment: radar params independent of spherical base ---
    {
        systems::sensors::radar::RadarEcho echo;
        echo.range = 500.0;
        echo.azimuth = 0.1;
        echo.elevation = -0.05;
        echo.signature = 2.0;
        echo.detectibility = 0.8;
        echo.likelihoodOfDetect = 0.75;

        assert(echo.range == 500.0);
        assert(echo.azimuth == 0.1);
        assert(echo.elevation == -0.05);
        assert(echo.signature == 2.0);
        assert(echo.detectibility == 0.8);
        assert(echo.likelihoodOfDetect == 0.75);
    }

    return 0;
}
