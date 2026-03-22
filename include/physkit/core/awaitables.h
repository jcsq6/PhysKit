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
struct wait_for
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_for aw)
            : detail::awaiter(promise), duration(aw.duration), start_time(world().time())
        {
        }

        mp_units::quantity<mp_units::si::second> duration;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept
        { return duration <= 0.0 * mp_units::si::second; }
        void await_suspend(std::coroutine_handle<detail::task_promise> handle)
        { scheduler().after(handle.promise().id, duration); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return world().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> duration;
};

struct wait_until
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, wait_until aw)
            : detail::awaiter(promise), time{aw.time}, start_time(world().time())
        {
        }
        mp_units::quantity<mp_units::si::second> time;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept { return time <= start_time; }
        void await_suspend(std::coroutine_handle<detail::task_promise> handle)
        { scheduler().at(handle.promise().id, time); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return world().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> time;
};

struct next_frame
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise &promise, next_frame /*unused*/)
            : detail::awaiter(promise), start_time(world().time())
        {
        }

        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] static bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<detail::task_promise> handle)
        { queue_task(handle.promise().id); }
        [[nodiscard]] mp_units::quantity<mp_units::si::second> await_resume() const noexcept
        { return world().time() - start_time; }
    };
};

} // namespace physkit