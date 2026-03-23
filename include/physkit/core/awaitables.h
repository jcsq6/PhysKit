#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "world.h"
#endif

PHYSKIT_EXPORT
namespace physkit
{
// --------- TEMPORAL ---------
struct wait_for
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_for aw)
            : detail::awaiter(promise), duration(aw.duration), start_time(handler().time())
        {
        }

        mp_units::quantity<mp_units::si::second> duration;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept
        { return duration <= 0.0 * mp_units::si::second; }
        void await_suspend(std::coroutine_handle<detail::task_promise> handle)
        { handler().schedule_task_after(handle.promise().id, duration); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return handler().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> duration;
};

struct wait_until
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_until aw)
            : detail::awaiter(promise), time{aw.time}, start_time(handler().time())
        {
        }
        mp_units::quantity<mp_units::si::second> time;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept { return time <= start_time; }
        void await_suspend(std::coroutine_handle<detail::task_promise> handle)
        { handler().schedule_task_at(handle.promise().id, time); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return handler().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> time;
};

struct next_frame
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, next_frame /*unused*/)
            : detail::awaiter(promise), start_time(handler().time())
        {
        }

        mp_units::quantity<mp_units::si::second> start_time;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<detail::task_promise> /**/)
        { handler().queue_task(promise().id); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return handler().time() - start_time; }
    };
};

// --------- COLLISIONS ---------
class one_sided_manifold
{
public:
    struct contact_pt
    {
        vec3<one> normal;
        vec3<si::metre> local_point;
        vec3<si::metre> local_point_other;
        quantity<si::metre> depth{};
    };

    one_sided_manifold() = default;
    one_sided_manifold(const detail::manifold &man, bool flip)
    {
        auto sign = flip ? -1.0 : 1.0;
        for (const auto &contact : man.contacts())
            M_contact_buffer[M_contacts++] = {
                .normal = flip ? -contact.point.normal : contact.point.normal,
                .local_point = flip ? contact.point.local_b : contact.point.local_a,
                .local_point_other = flip ? contact.point.local_a : contact.point.local_b,
                .depth = contact.point.depth,
            };
    }

    [[nodiscard]] auto contacts(this auto &&self)
    { return std::span{self.M_contact_buffer.data(), self.M_contacts}; }

private:
    std::array<contact_pt, detail::manifold::max_contact_points> M_contact_buffer{};
    std::size_t M_contacts{};
};

struct wait_for_collision
{
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-member-init)
    struct result_type
    {
        world_base::handle other;
        one_sided_manifold contact_manifold;
    };

    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_for_collision aw)
            : detail::awaiter(promise), object(aw.object)
        {
        }

        world_base::handle object;
        detail::narrow_phase::manifold_info result;

        [[nodiscard]] bool await_ready() const { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<detail::task_promise> /*handle*/)
        { handler().add_collision_waiter(object.id(), promise().id, &result); }
        [[nodiscard]] result_type await_resume() const noexcept
        {
            if (result.a == object.index())
                return {.other = world_base::handle::from_id(result.b),
                        .contact_manifold = {result.man, false}};
            return {.other = world_base::handle::from_id(result.a),
                    .contact_manifold = {result.man, true}};
        }
    };

    world_base::handle object;
};

struct wait_for_collision_exit
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_for_collision_exit aw)
            : detail::awaiter(promise), object(aw.object)
        {
        }

        world_base::handle object;
        detail::handle_id_t result_a = world_base::handle::null;
        detail::handle_id_t result_b = world_base::handle::null;

        // TODO: maybe make sure it's actually colliding?
        [[nodiscard]] bool await_ready() const { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<detail::task_promise> /*handle*/)
        { handler().add_collision_exit_waiter(object.id(), promise().id, &result_a, &result_b); }
        [[nodiscard]] world_base::handle await_resume() const noexcept
        { return world_base::handle::from_id((result_a == object.id()) ? result_b : result_a); }
    };

    world_base::handle object;
};

// --------- KINEMATICS ---------
// TODO:
// wait_until_sleeping(object)
// wait_until_awake(object)

// --------- CONSTRAINTS ---------
// TODO:
// wait_until_impulse_applied(object, constraint)

// --------- STRUCTURAL ---------
// TODO:
// wait_until_destroyed(object)
// threading...

} // namespace physkit