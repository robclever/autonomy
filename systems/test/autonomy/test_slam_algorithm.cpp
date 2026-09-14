#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

#include <autonomy/SLAM/EkfSlam.h>
#include <autonomy/SLAM/GraphSlam.h>
#include <autonomy/SLAM/SlamAlgorithm.h>

namespace
{
constexpr double kTol = 1e-9;
void expectNear(double a, double b, double tol = kTol) { assert(std::fabs(a - b) <= tol); }

// Build a measurement with a known landmark id.
systems::autonomy::SlamMeasurement makeMeas(double range, double az, double el, std::uint32_t id)
{
    systems::autonomy::SlamMeasurement m;
    m.range = range;
    m.azimuth = az;
    m.elevation = el;
    m.landmarkId = id;
    return m;
}
} // namespace

// Unit tests for the SLAM algorithm interface and stub implementations.
int main()
{
    using systems::autonomy::EkfSlam;
    using systems::autonomy::GraphSlam;
    using systems::autonomy::SlamAlgorithm;
    using systems::autonomy::SlamInput;
    using systems::autonomy::SlamState;

    // --- Polymorphic use: both algorithms behind SlamAlgorithm* ---
    {
        std::unique_ptr<SlamAlgorithm> algos[] = {std::make_unique<EkfSlam>(),
                                                  std::make_unique<GraphSlam>()};

        for (auto& algo : algos)
        {
            SlamInput input;
            input.pose.position = core::math::Vec3{1.0, 2.0, -3.0};
            input.measurements = {makeMeas(100.0, 0.1, 0.0, 1), makeMeas(200.0, -0.2, 0.05, 2)};

            algo->process(input);
            SlamState s = algo->state();

            // Pose recorded.
            expectNear(s.pose.position.x, 1.0);
            expectNear(s.pose.position.y, 2.0);
            expectNear(s.pose.position.z, -3.0);

            // Two landmarks added.
            assert(s.landmarks.size() == 2);
            assert(s.landmarks[0].id == 1);
            assert(s.landmarks[1].id == 2);

            // Reset clears state.
            algo->reset();
            s = algo->state();
            assert(s.landmarks.empty());
        }
    }

    // --- Measurements without landmarkId are ignored by stub ---
    {
        EkfSlam ekf;
        SlamInput input;
        input.pose.position = core::math::Vec3{0.0, 0.0, 0.0};
        systems::autonomy::SlamMeasurement m;
        m.range = 50.0;
        m.landmarkId = std::nullopt; // unknown association
        input.measurements = {m};

        ekf.process(input);
        assert(ekf.state().landmarks.empty());
    }

    // --- Duplicate landmark ids are not double-added ---
    {
        GraphSlam graph;
        SlamInput input;
        input.pose.position = core::math::Vec3{0.0, 0.0, 0.0};
        input.measurements = {makeMeas(100.0, 0.0, 0.0, 5), makeMeas(105.0, 0.01, 0.0, 5)};

        graph.process(input);
        assert(graph.state().landmarks.size() == 1);
        assert(graph.state().landmarks[0].id == 5);
    }

    return 0;
}
