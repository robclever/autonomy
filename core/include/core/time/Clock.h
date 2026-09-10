#pragma once

#include <cstdint>

namespace core
{
namespace time
{

// Deterministic, step-based clock. The whole simulation advances in fixed
// timesteps so runs are reproducible and unit-testable (see .README,
// "Guiding principles").
class Clock
{
  public:
    using duration = std::uint64_t; // ticks
    using time = std::uint64_t;     // ticks since simulation start

    explicit Clock(duration step_ns, time start_ns = 0) noexcept
        : step_ns_(step_ns), now_ns_(start_ns)
    {
    }

    void advance() noexcept { now_ns_ += step_ns_; }
    void advance(duration n_steps) noexcept { now_ns_ += step_ns_ * n_steps; }

    time now_ns() const noexcept { return now_ns_; }
    duration step_ns() const noexcept { return step_ns_; }

  private:
    duration step_ns_;
    time now_ns_;
};

} // namespace time
} // namespace core