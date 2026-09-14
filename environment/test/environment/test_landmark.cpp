#include <cassert>
#include <string>

#include <Landmark.h>

// Unit tests for environment/Landmark.
int main()
{
    using core::math::Vec3;
    using sim::Landmark;

    // --- Default construction ---
    {
        Landmark lm;
        assert(lm.id == 0);
        assert(lm.name.empty());
        assert(lm.positionNed.x == 0.0);
        assert(lm.positionNed.y == 0.0);
        assert(lm.positionNed.z == 0.0);
        assert(lm.reflectivity == 1.0);
    }

    // --- Field assignment ---
    {
        Landmark lm;
        lm.id = 42;
        lm.name = "test_tower";
        lm.positionNed = Vec3{100.0, 200.0, -50.0};
        lm.reflectivity = 0.75;

        assert(lm.id == 42);
        assert(lm.name == "test_tower");
        assert(lm.positionNed.x == 100.0);
        assert(lm.positionNed.y == 200.0);
        assert(lm.positionNed.z == -50.0);
        assert(lm.reflectivity == 0.75);
    }

    return 0;
}
