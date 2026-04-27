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
static constexpr auto frame_ds = 5.0 * ms;

static auto no_gravity() { return world_desc::make().with_gravity(vec3{0, 0, 0} * m / s / s); }

const auto box_mesh = box(vec3{1, 1, 1} * m);

// =====================================================================
// Temporal awaitables
// =====================================================================

void wait()
{
    world w(no_gravity());

    auto t = []() -> task<>
    {
        auto elapsed = *co_await wait_for(1.0 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);

        elapsed = *co_await next_frame{};
        CHECK_APPROX(elapsed, frame_ds.in(s));

        elapsed = *co_await wait_until_time(2 * s);
        CHECK_APPROX(elapsed, 1.0 * s, time_eps);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
}

void wait_for_zero_duration()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // Zero duration should be ready immediately (await_ready returns true)
        auto elapsed = *co_await wait_for(0.0 * s);
        CHECK_APPROX(elapsed, 0.0 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(completed);
}

void wait_for_negative_duration()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // Negative duration should be ready immediately
        auto elapsed = *co_await wait_for(-1.0 * s);
        CHECK_APPROX(elapsed, 0.0 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(completed);
}

void wait_until_past_time()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // First advance the clock
        co_await wait_for(1.0 * s);
        // Then wait_until_time a time already passed — should be ready immediately
        auto elapsed = *co_await wait_until_time(0.5 * s);
        CHECK_APPROX(elapsed, 0.0 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(completed);
}

void wait_for_multiple_sequential()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        auto e1 = *co_await wait_for(0.5 * s);
        CHECK_APPROX(e1, 0.5 * s, time_eps);

        auto e2 = *co_await wait_for(0.3 * s);
        CHECK_APPROX(e2, 0.3 * s, time_eps);

        auto e3 = *co_await wait_for(0.2 * s);
        CHECK_APPROX(e3, 0.2 * s, time_eps);

        // Total elapsed should be 1.0s
        auto e4 = *co_await wait_until_time(1.0 * s);
        CHECK_APPROX(e4, 0.0 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(completed);
}

void next_physics_tick_test()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // next_physics_tick resumes in the post-physics phase of the same step,
        // so elapsed within a single step is 0
        auto elapsed = *co_await next_physics_tick{};
        CHECK_APPROX(elapsed, 0.0 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    w.step(frame_ds);
    CHECK(completed);
}

void multiple_next_frames()
{
    world w(no_gravity());

    static int frame_count = 0;
    frame_count = 0;

    auto t = []() -> task<>
    {
        for (int i = 0; i < 10; ++i)
        {
            co_await next_frame{};
            ++frame_count;
        }
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(frame_count == 10);
}

void multiple_timer_ordering()
{
    world w(no_gravity());

    static std::vector<int> order;
    order.clear();

    auto t1 = []() -> task<>
    {
        co_await wait_for(0.3 * s);
        order.push_back(2);
    };
    auto t2 = []() -> task<>
    {
        co_await wait_for(0.1 * s);
        order.push_back(1);
    };
    auto t3 = []() -> task<>
    {
        co_await wait_for(0.5 * s);
        order.push_back(3);
    };

    w.add_task(t1());
    w.add_task(t2());
    w.add_task(t3());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);

    CHECK(order.size() == 3);
    CHECK(order[0] == 1);
    CHECK(order[1] == 2);
    CHECK(order[2] == 3);
}

// =====================================================================
// Subtasks and task<T>
// =====================================================================

static task<int> make_value(int val)
{
    co_await wait_for(0.5 * s);
    co_return val;
}

void subtask()
{
    world w(no_gravity());

    auto t = []() -> task<>
    {
        auto val = *co_await make_value(42);
        CHECK(val == 42);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
}

void nested_subtask()
{
    world w(no_gravity());

    auto t = []() -> task<>
    {
        auto a = *co_await make_value(10);
        co_await wait_for(0.5 * s);
        auto b = *co_await make_value(20);
        CHECK(a + b == 30);

        // Verify timing: 0.5 (first make_value) + 0.5 (wait_for) + 0.5 (second make_value) = 1.5s
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
}

void subtask_void()
{
    world w(no_gravity());

    static bool inner_ran = false;
    inner_ran = false;

    auto inner = []() -> task<>
    {
        co_await wait_for(0.5 * s);
        inner_ran = true;
    };

    // NOLINTNEXTLINE
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

static task<int> failing_subtask()
{
    co_await wait_for(0.1 * s);
    throw std::runtime_error("subtask failure");
    co_return 0; // unreachable
}

void exception_propagation()
{
    world w(no_gravity());

    static bool caught = false;
    caught = false;

    auto t = []() -> task<>
    {
        auto res = co_await failing_subtask();
        CHECK(!res.has_value());
        CHECK(res.error().value == error::exception_thrown);
        try
        {
            std::rethrow_exception(res.error().exception);
        }
        catch (const std::runtime_error &e)
        {
            CHECK(std::string_view(e.what()) == "subtask failure");
            caught = true;
        }
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(caught);
}

static task<std::string> deeply_nested_value()
{
    auto val = *co_await make_value(7);
    co_return std::to_string(val) + "_nested";
}

void deep_subtask_chain()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        auto result = *co_await deeply_nested_value();
        CHECK(result == "7_nested");
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
    CHECK(completed);
}

// =====================================================================
// Collision awaitables
// =====================================================================

void collision()
{
    world w(no_gravity());
    auto a = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{1, 0, 0} * m / s)
                                .with_pos(vec3{-1, 0, 0} * m)
                                .with_shape(box_mesh));
    auto b = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{-1, 0, 0} * m / s)
                                .with_pos(vec3{1, 0, 0} * m)
                                .with_shape(box_mesh));
    auto t = [](object_handle a, object_handle b) -> task<>
    {
        auto col = *co_await wait_for_collision(a);
        CHECK(col.other == b);
        CHECK(col.contact_manifold.contacts().size() > 0);
        auto other = *co_await wait_for_separation(b);
        CHECK(other == a);
    };
    w.add_task(t(a, b));

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
}

void collision_with_specific_object()
{
    world w(no_gravity());
    auto a = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{1, 0, 0} * m / s)
                                .with_pos(vec3{-1, 0, 0} * m)
                                .with_shape(box_mesh));
    auto b = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{-1, 0, 0} * m / s)
                                .with_pos(vec3{1, 0, 0} * m)
                                .with_shape(box_mesh));
    auto c = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{0, 0, 0} * m / s)
                                .with_pos(vec3{0, 10, 0} * m) // Far away
                                .with_shape(box_mesh));

    static bool completed_ab = false;
    static bool completed_ac = false;
    completed_ab = false;
    completed_ac = false;

    auto t = [](object_handle a, object_handle b) -> task<>
    {
        auto col = *co_await wait_for_collision(a, b);
        CHECK(col.other == b);
        CHECK(col.contact_manifold.contacts().size() > 0);

        auto other = *co_await wait_for_separation(b, a);
        CHECK(other == a);

        completed_ab = true;
    };

    auto t2 = [](object_handle a, object_handle c) -> task<>
    {
        auto result = *co_await wait_for_any{wait_for_collision(a, c), wait_for(4 * s)};
        completed_ac = result.index() != 1;
    };

    w.add_task(t(a, b));
    w.add_task(t2(a, c));

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);

    CHECK(completed_ab);
    CHECK(!completed_ac);
}

void collision_stale_handle()
{
    world w(no_gravity());

    // Create then destroy to get a stale (generation-mismatched) handle
    auto stale =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));
    w.remove_rigid(stale);

    static bool caught = false;
    caught = false;

    auto t = [](object_handle h) -> task<>
    {
        auto result = co_await wait_for_collision(h);
        CHECK(!result.has_value());
        CHECK(result.error().value == error::stale_handle);
        caught = true;
    };
    w.add_task(t(stale));

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(caught);
}

void separation_stale_handle()
{
    world w(no_gravity());

    auto stale =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));
    w.remove_rigid(stale);

    static bool caught = false;
    caught = false;

    auto t = [](object_handle h) -> task<>
    {
        auto result = co_await wait_for_separation(h);
        CHECK(!result.has_value());
        CHECK(result.error().value == error::stale_handle);
        caught = true;
    };
    w.add_task(t(stale));

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(caught);
}

// =====================================================================
// wait_until_destroyed
// =====================================================================

void destroyed_awaitable()
{
    world w(no_gravity());

    auto obj =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));

    static bool completed = false;
    completed = false;

    auto t = [](object_handle h) -> task<>
    {
        *co_await wait_until_destroyed(h);
        completed = true;
    };

    auto destroyer = [](object_handle h) -> task<>
    {
        co_await wait_for(0.5 * s);
        co_await destroy_rigid(h);
    };

    w.add_task(t(obj));
    w.add_task(destroyer(obj));

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(completed);
}

void destroyed_stale_handle()
{
    world w(no_gravity());

    auto stale =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));
    w.remove_rigid(stale);

    static bool caught = false;
    caught = false;

    auto t = [](object_handle h) -> task<>
    {
        auto result = co_await wait_until_destroyed(h);
        CHECK(!result.has_value());
        CHECK(result.error().value == error::stale_handle);
        caught = true;
    };
    w.add_task(t(stale));

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(caught);
}

// =====================================================================
// wait_for_all
// =====================================================================

void wait_for_all_temporal()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // wait_for_all with different durations — should complete after the longest
        auto [e1, e2, e3] =
            co_await wait_for_all{wait_for(0.2 * s), wait_for(0.5 * s), wait_for(0.1 * s)};
        CHECK_APPROX(*e1, 0.2 * s, time_eps);
        CHECK_APPROX(*e2, 0.5 * s, time_eps);
        CHECK_APPROX(*e3, 0.1 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(completed);
}

void wait_for_all_with_subtasks()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        auto [a, b] = co_await wait_for_all{make_value(10), make_value(20)};
        CHECK(a == 10);
        CHECK(b == 20);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
    CHECK(completed);
}

static task<> void_wait_task() { co_await wait_for(0.3 * s); }

void wait_for_all_void_and_value()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // Mix void and valued awaitables
        auto [_, val] = co_await wait_for_all{void_wait_task(), make_value(42)};
        CHECK(val == 42);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
    CHECK(completed);
}

void wait_for_all_exception()
{
    world w(no_gravity());

    static bool caught = false;
    caught = false;

    auto t = []() -> task<>
    {
        auto tuple_res = co_await wait_for_all{failing_subtask(), make_value(10)};
        auto &fail_res = std::get<0>(tuple_res);
        CHECK(!fail_res.has_value());
        CHECK(fail_res.error().value == error::exception_thrown);
        try
        {
            std::rethrow_exception(fail_res.error().exception);
        }
        catch (const std::runtime_error &e)
        {
            CHECK(std::string_view(e.what()) == "subtask failure");
            caught = true;
        }
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(caught);
}

// =====================================================================
// wait_for_any
// =====================================================================

void wait_for_any_temporal()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // The shortest wait should win
        auto result =
            *co_await wait_for_any{wait_for(1.0 * s), wait_for(0.1 * s), wait_for(0.5 * s)};
        // Index 1 (the 0.1s wait) should complete first
        CHECK(result.index() == 1);
        CHECK_APPROX(std::get<1>(result), 0.1 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(completed);
}

static task<int> slow_value()
{
    co_await wait_for(2.0 * s);
    co_return 999;
}

void wait_for_any_with_subtasks()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // make_value takes 0.5s, slow_value takes 2.0s
        auto result = *co_await wait_for_any{slow_value(), make_value(42)};
        CHECK(result.index() == 1);
        CHECK(std::get<1>(result) == 42);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(3 * s);
    CHECK(completed);
}

void wait_for_any_all_fail()
{
    world w(no_gravity());

    auto t = []() -> task<>
    {
        auto result_expected = co_await wait_for_any{failing_subtask(), failing_subtask()};
        CHECK(!result_expected.has_value());
        CHECK(result_expected.error().value == error::exception_thrown);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
}

void wait_for_any_collision_or_timeout()
{
    world w(no_gravity());

    // Objects far apart — they won't collide within the timeout
    auto a =
        w.create_rigid(object_desc::dynam().with_pos(vec3{-100, 0, 0} * m).with_shape(box_mesh));

    static bool completed = false;
    completed = false;

    auto t = [](object_handle a) -> task<>
    {
        // Timeout should win since objects never collide
        auto result = *co_await wait_for_any{wait_for_collision(a), wait_for(0.2 * s)};
        CHECK(result.index() == 1); // timeout won
        completed = true;
    };
    w.add_task(t(a));

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(completed);
}

// =====================================================================
// Command awaitables
// =====================================================================

void create_rigid_command()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        auto h = *co_await create_rigid(
            object_desc::dynam().with_pos(vec3{5, 5, 5} * m).with_shape(box_mesh));
        auto *obj = *co_await get_rigid(h);
        CHECK_APPROX(obj->pos().x(), 5.0 * m);
        CHECK_APPROX(obj->pos().y(), 5.0 * m);
        CHECK_APPROX(obj->pos().z(), 5.0 * m);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(completed);
}

void destroy_rigid_command()
{
    world w(no_gravity());

    auto obj =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));

    static bool completed = false;
    completed = false;

    auto t = [](object_handle h) -> task<>
    {
        auto result = co_await destroy_rigid(h);
        CHECK(result.has_value());
        // Object should no longer exist
        auto rigid = co_await get_rigid(h);
        CHECK(!rigid.has_value());
        CHECK(rigid.error().value == error::stale_handle);
        completed = true;
    };
    w.add_task(t(obj));

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(completed);
}

void get_rigid_stale()
{
    world w(no_gravity());

    auto stale =
        w.create_rigid(object_desc::dynam().with_pos(vec3{0, 0, 0} * m).with_shape(box_mesh));
    w.remove_rigid(stale);

    static bool completed = false;
    completed = false;

    auto t = [](object_handle h) -> task<>
    {
        auto rigid = co_await get_rigid(h);
        CHECK(!rigid.has_value());
        CHECK(rigid.error().value == error::stale_handle);
        completed = true;
    };
    w.add_task(t(stale));

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(completed);
}

void create_and_destroy_in_coroutine()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // Create, verify, destroy, verify gone — full lifecycle
        auto h = *co_await create_rigid(
            object_desc::dynam().with_pos(vec3{1, 2, 3} * m).with_shape(box_mesh));
        CHECK((co_await get_rigid(h)).has_value());

        co_await destroy_rigid(h);
        CHECK(!(co_await get_rigid(h)).has_value());
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(completed);
}

void add_task_command_test()
{
    world w(no_gravity());

    static bool spawned_ran = false;
    spawned_ran = false;

    static auto spawned = []() -> task<>
    {
        co_await wait_for(0.2 * s);
        spawned_ran = true;
    };

    auto t = []() -> task<>
    {
        // add_task with policy::wait waits for the spawned task to complete
        co_await physkit::add_task(spawned());
        CHECK(spawned_ran);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(spawned_ran);
}

void add_task_no_wait_test()
{
    world w(no_gravity());

    static bool spawned_ran = false;
    spawned_ran = false;
    static bool parent_continued = false;
    parent_continued = false;

    static auto spawned = []() -> task<>
    {
        co_await wait_for(0.5 * s);
        spawned_ran = true;
    };

    auto t = []() -> task<>
    {
        // no_wait: parent continues immediately
        co_await physkit::add_task<policy::no_wait>(spawned());
        parent_continued = true;
        co_await wait_for(1.0 * s);
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(2 * s);
    CHECK(parent_continued);
    CHECK(spawned_ran);
}

// =====================================================================
// Concurrent tasks
// =====================================================================

void concurrent_independent_tasks()
{
    world w(no_gravity());

    static int count = 0;
    count = 0;

    auto t = []() -> task<>
    {
        co_await wait_for(0.5 * s);
        ++count;
    };

    // Launch 5 independent tasks
    for (int i = 0; i < 5; ++i) w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(count == 5);
}

void task_completes_immediately()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    static auto instant = []() -> task<int> { co_return 42; };

    auto t = []() -> task<>
    {
        auto val = co_await instant();
        CHECK(val == 42);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(completed);
}

// =====================================================================
// Mixed composition
// =====================================================================

void wait_for_any_mixed_with_physics_tick()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // next_physics_tick should fire each frame, winning over a long wait
        auto result = *co_await wait_for_any{wait_for(10.0 * s), next_physics_tick{}};
        CHECK(result.index() == 1);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(frame_ds);
    CHECK(completed);
}

void wait_for_all_with_next_frame()
{
    world w(no_gravity());

    static bool completed = false;
    completed = false;

    auto t = []() -> task<>
    {
        // Both should complete: next_frame in one frame, wait_for in many
        auto [frame_elapsed, wait_elapsed] = co_await wait_for_all{next_frame{}, wait_for(0.1 * s)};
        CHECK_APPROX(*frame_elapsed, frame_ds.in(s));
        CHECK_APPROX(*wait_elapsed, 0.1 * s, time_eps);
        completed = true;
    };
    w.add_task(t());

    stepper stepper(w, frame_ds);
    stepper.update(1 * s);
    CHECK(completed);
}

void destroyed_triggers_during_collision()
{
    // Create two colliding objects, wait for collision on one while also
    // watching for destruction on the other
    world w(no_gravity());

    auto a = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{1, 0, 0} * m / s)
                                .with_pos(vec3{-1, 0, 0} * m)
                                .with_shape(box_mesh));
    auto b = w.create_rigid(object_desc::dynam()
                                .with_vel(vec3{-1, 0, 0} * m / s)
                                .with_pos(vec3{1, 0, 0} * m)
                                .with_shape(box_mesh));

    static bool collided = false;
    collided = false;
    static bool destroyed = false;
    destroyed = false;

    auto collision_task = [](object_handle h) -> task<>
    {
        co_await wait_for_collision(h);
        collided = true;
    };

    auto destroy_task = [](object_handle target) -> task<>
    {
        // Destroy after collision has time to happen
        co_await wait_for(2.0 * s);
        co_await destroy_rigid(target);
    };

    auto wait_destroy_task = [](object_handle target) -> task<>
    {
        co_await wait_until_destroyed(target);
        destroyed = true;
    };

    w.add_task(collision_task(a));
    w.add_task(wait_destroy_task(b));
    w.add_task(destroy_task(b));

    stepper stepper(w, frame_ds);
    stepper.update(5 * s);
    CHECK(collided);
    CHECK(destroyed);
}

} // namespace tests

int main()
{
    suite s;

    // Temporal
    s.group("Temporal waits").test("basic waits", tests::wait);
    s.group("Temporal waits").test("zero duration", tests::wait_for_zero_duration);
    s.group("Temporal waits").test("negative duration", tests::wait_for_negative_duration);
    s.group("Temporal waits").test("wait_until_time past time", tests::wait_until_past_time);
    s.group("Temporal waits").test("sequential waits", tests::wait_for_multiple_sequential);
    s.group("Temporal waits").test("next_physics_tick", tests::next_physics_tick_test);
    s.group("Temporal waits").test("multiple next_frames", tests::multiple_next_frames);
    s.group("Temporal waits").test("timer ordering", tests::multiple_timer_ordering);

    // Subtasks
    s.group("Subtasks").test("value return", tests::subtask);
    s.group("Subtasks").test("nested subtask", tests::nested_subtask);
    s.group("Subtasks").test("void subtask", tests::subtask_void);
    s.group("Subtasks").test("exception propagation", tests::exception_propagation);
    s.group("Subtasks").test("deep chain", tests::deep_subtask_chain);
    s.group("Subtasks").test("immediate completion", tests::task_completes_immediately);

    // Collisions
    s.group("Collisions").test("collision and separation", tests::collision);
    s.group("Collisions").test("specific object collision", tests::collision_with_specific_object);
    s.group("Collisions").test("stale handle collision", tests::collision_stale_handle);
    s.group("Collisions").test("stale handle separation", tests::separation_stale_handle);

    // Destruction
    s.group("Destruction").test("wait_until_destroyed", tests::destroyed_awaitable);
    s.group("Destruction").test("stale handle destroyed", tests::destroyed_stale_handle);

    // wait_for_all
    s.group("wait_for_all").test("temporal", tests::wait_for_all_temporal);
    s.group("wait_for_all").test("subtasks", tests::wait_for_all_with_subtasks);
    s.group("wait_for_all").test("void and value mix", tests::wait_for_all_void_and_value);
    s.group("wait_for_all").test("exception", tests::wait_for_all_exception);
    s.group("wait_for_all").test("with next_frame", tests::wait_for_all_with_next_frame);

    // wait_for_any
    s.group("wait_for_any").test("temporal race", tests::wait_for_any_temporal);
    s.group("wait_for_any").test("subtask race", tests::wait_for_any_with_subtasks);
    s.group("wait_for_any").test("all fail", tests::wait_for_any_all_fail);
    s.group("wait_for_any").test("collision vs timeout", tests::wait_for_any_collision_or_timeout);
    s.group("wait_for_any").test("with physics tick", tests::wait_for_any_mixed_with_physics_tick);

    // Commands
    s.group("Commands").test("create_rigid", tests::create_rigid_command);
    s.group("Commands").test("destroy_rigid", tests::destroy_rigid_command);
    s.group("Commands").test("get_rigid stale", tests::get_rigid_stale);
    s.group("Commands")
        .test("create and destroy lifecycle", tests::create_and_destroy_in_coroutine);
    s.group("Commands").test("add_task wait", tests::add_task_command_test);
    s.group("Commands").test("add_task no_wait", tests::add_task_no_wait_test);

    // Concurrent / mixed
    s.group("Concurrent").test("independent tasks", tests::concurrent_independent_tasks);
    s.group("Concurrent")
        .test("destruction during collision", tests::destroyed_triggers_during_collision);

    return s.run();
}
