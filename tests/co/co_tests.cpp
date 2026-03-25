#define PHYSKIT_NO_DEBUG_LIMITER
#include "test.h"

#ifndef PHYSKIT_MODULES
// #include <print>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace testing;

namespace tests
{
static constexpr auto time_eps = .01 * s;

const auto box_mesh = mesh::box(vec3{1, 1, 1} * m);
void wait()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    auto t = []() -> task<>
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

static task<int> make_value(int val)
{
    (void) co_await wait_for(0.5 * s);
    co_return val;
}

void subtask()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    auto t = []() -> task<>
    {
        auto val = co_await make_value(42);
        CHECK(val == 42);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
}

void nested_subtask()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    auto t = []() -> task<>
    {
        auto a = co_await make_value(10);
        (void) co_await wait_for(0.5 * s);
        auto b = co_await make_value(20);
        CHECK(a + b == 30);

        // Verify timing: 0.5 (first make_value) + 0.5 (wait_for) + 0.5 (second make_value) = 1.5s
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
}

void subtask_void()
{
    static constexpr auto frame_ds = 5.0 * ms;
    world_desc desc = world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s);
    world<semi_implicit_euler> w(desc);

    static bool inner_ran = false;
    inner_ran = false;

    auto inner = []() -> task<>
    {
        (void) co_await wait_for(0.5 * s);
        inner_ran = true;
    };

    auto outer = [&]() -> task<>
    {
        CHECK(!inner_ran);
        co_await inner();
        CHECK(inner_ran);
    };
    w.add_task(outer());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
    CHECK(inner_ran);
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
    auto t = [](object_handle a, object_handle b) -> task<>
    {
        auto col = co_await wait_for_collision(a);
        CHECK(col.other == b);
        CHECK(col.contact_manifold.contacts().size() > 0);
        auto other = co_await wait_for_separation(b);
        CHECK(other == a);
    };
    w.add_task(t(a, b));

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
}
} // namespace tests

int main()
{
    suite s;

    s.group("Coroutine waits").test("wait", tests::wait);
    s.group("Coroutine waits").test("collision", tests::collision);
    s.group("Coroutine subtasks").test("subtask", tests::subtask);
    s.group("Coroutine subtasks").test("nested_subtask", tests::nested_subtask);
    s.group("Coroutine subtasks").test("subtask_void", tests::subtask_void);
    return s.run();
}
