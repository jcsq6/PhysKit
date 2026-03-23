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

const auto box_mesh = mesh::box(vec3{1, 1, 1} * m);

void wait()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    auto t = []() -> task
    {
        auto elapsed = co_await wait_for(1.0 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);

        elapsed = co_await next_frame{};
        CHECK_APPROX(elapsed, frame_ds.in(s));

        elapsed = co_await wait_until(2 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
}

void collision()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);
    auto a = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{1, 0, 0} * m / s)
                                .with_pos(vec3{-1, 0, 0} * m)
                                .with_mesh(box_mesh));
    auto b = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{-1, 0, 0} * m / s)
                                .with_pos(vec3{1, 0, 0} * m)
                                .with_mesh(box_mesh));
    auto t = [](object_handle a, object_handle b) -> task
    {
        auto col = co_await wait_for_collision(a);
        CHECK(col.other == b);
        CHECK(col.contact_manifold.contacts().size() > 0);
        auto other = co_await wait_for_collision_exit(b);
        CHECK(other == a);
    };
    w.add_task(t(a, b));

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
}

int main()
{
    suite s;

    s.group("Coroutine waits").test("wait", wait);
    s.group("Coroutine waits").test("collision", collision);
    return s.run();
}
