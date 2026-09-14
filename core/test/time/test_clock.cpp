#include <cassert>

#include <time/Clock.h>

// Minimal unit test for core/Clock. Uses assert() directly for now; a test
// framework (Catch2 / GoogleTest) can be wired in later.
int main()
{
    using namespace core::time;

    Clock clock(10'000'000); // 10 ms steps
    assert(clock.now_ns() == 0);
    assert(clock.step_ns() == 10'000'000);

    clock.advance();
    assert(clock.now_ns() == 10'000'000);

    clock.advance(3);
    assert(clock.now_ns() == 40'000'000);

    return 0;
}