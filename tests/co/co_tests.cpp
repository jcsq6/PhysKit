#define PHYSKIT_NO_DEBUG_LIMITER
#include "test.h"

#ifndef PHYSKIT_MODULES
// #include <print>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

static constexpr auto time_eps = .01 * s;

void wait()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    auto t = [&](this auto) -> task
    {
        auto elapsed = co_await wait_for(1.0 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);

        elapsed = co_await next_frame{};
        CHECK_APPROX(elapsed, frame_ds.in(s));

        elapsed = co_await wait_until(2 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);
    }();
    w.add_task(std::move(t));

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
}

int main()
{
    suite s;

    s.group("Coroutine waits").test("wait", wait);

    return s.run();
}
