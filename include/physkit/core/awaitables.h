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
        awaiter_type(detail::task_promise_base &promise, wait_for aw)
            : detail::awaiter(promise), duration(aw.duration), start_time(handler().time())
        {
        }

        mp_units::quantity<mp_units::si::second> duration;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept
        { return duration <= 0.0 * mp_units::si::second; }
        void await_suspend(std::coroutine_handle<> handle)
        { handler().schedule_task_after(promise().id, duration); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        mp_units::quantity<mp_units::si::second> on_resume() const noexcept
        { return handler().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> duration;
};

struct wait_until
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_until aw)
            : detail::awaiter(promise), time{aw.time}, start_time(handler().time())
        {
        }
        mp_units::quantity<mp_units::si::second> time;
        mp_units::quantity<mp_units::si::second> start_time;

        [[nodiscard]] bool await_ready() const noexcept { return time <= start_time; }
        void await_suspend(std::coroutine_handle<> handle)
        { handler().schedule_task_at(promise().id, time); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        mp_units::quantity<mp_units::si::second> on_resume() const noexcept
        { return handler().time() - start_time; }
    };

    mp_units::quantity<mp_units::si::second> time;
};

struct next_frame
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, next_frame /*unused*/)
            : detail::awaiter(promise), start_time(handler().time())
        {
        }

        mp_units::quantity<mp_units::si::second> start_time;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<> /**/) { handler().queue_pre_task(promise().id); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        mp_units::quantity<mp_units::si::second> on_resume() const noexcept
        { return handler().time() - start_time; }
    };
};

struct next_physics_tick
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, next_physics_tick /*unused*/)
            : detail::awaiter(promise), start_time(handler().time())
        {
        }

        mp_units::quantity<mp_units::si::second> start_time;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() noexcept { return false; }
        void await_suspend(std::coroutine_handle<> /**/)
        { handler().queue_post_physics_tick(promise().id); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        mp_units::quantity<mp_units::si::second> on_resume() const noexcept
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
        awaiter_type(detail::task_promise_base &promise, wait_for_collision aw)
            : detail::awaiter(promise), object(aw.object), with(aw.with)
        {
        }

        ~awaiter_type()
        {
            // cancelled
            if (is_suspended) handler().remove_collision_waiter(object.id(), promise().id);
        }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        world_base::handle object;
        world_base::handle with;
        detail::narrow_phase::manifold_info result;
        bool is_suspended = false;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() const noexcept { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            handler().add_collision_waiter(object.id(), with.id(), promise().id, &result);
            is_suspended = true;
        }

        result_type on_resume()
        {
            if (!is_suspended) throw error::stale_handle;
            is_suspended = false;
            if (result.a == object.index())
                return {.other = world_base::handle::from_id(result.b),
                        .contact_manifold = {result.man, false}};
            return {.other = world_base::handle::from_id(result.a),
                    .contact_manifold = {result.man, true}};
        }
    };

    world_base::handle object;
    world_base::handle with = world_base::handle::from_id(world_base::handle::null);
};

struct wait_for_separation
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_for_separation aw)
            : detail::awaiter(promise), object(aw.object), with(aw.with)
        {
        }

        ~awaiter_type()
        {
            if (is_suspended) handler().remove_collision_exit_waiter(object.id(), promise().id);
        }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        world_base::handle object;
        world_base::handle with;
        detail::handle_id_t result_a = world_base::handle::null;
        detail::handle_id_t result_b = world_base::handle::null;
        bool is_suspended = false;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() const noexcept { return !world().get_rigid(object); }

        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            handler().add_collision_exit_waiter(object.id(), with.id(), promise().id, &result_a,
                                                &result_b);
            is_suspended = true;
        }

        world_base::handle on_resume()
        {
            if (!is_suspended) throw error::stale_handle;
            is_suspended = false;
            return world_base::handle::from_id((result_a == object.id()) ? result_b : result_a);
        }
    };

    world_base::handle object;
    world_base::handle with = world_base::handle::from_id(world_base::handle::null);
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
// threading...

struct wait_until_destroyed
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_until_destroyed aw)
            : detail::awaiter(promise), object(aw.object)
        {
        }

        ~awaiter_type()
        {
            if (is_suspended) handler().remove_destruction_waiter(object.id(), promise().id);
        }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        world_base::handle object;
        bool is_suspended = false;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() const noexcept { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            is_suspended = true;
            handler().add_destruction_waiter(object.id(), promise().id);
        }

        void on_resume()
        {
            if (!is_suspended) throw error::stale_handle;
            is_suspended = false;
        }
    };

    world_base::handle object;
};

// --------- META ---------

namespace detail
{
struct completion_barrier
{
    std::atomic<std::size_t> remaining; // atomic to prepare for threading support
    task_id parent;
    task_handler *handler;

    void arrive()
    {
        if (--remaining == 0) handler->queue_immediate_task(parent);
    }
};

struct race_barrier
{
    std::atomic<bool> finished{false};
    std::atomic<std::size_t> failures{0};
    std::size_t total;
    task_id parent;
    task_handler *handler;

    bool arrive()
    {
        bool expected = false;
        if (finished.compare_exchange_strong(expected, true))
        {
            handler->queue_immediate_task(parent);
            return true;
        }
        return false;
    }

    void arrive_error()
    {
        if (++failures == total)
        {
            bool expected = false;
            if (finished.compare_exchange_strong(expected, true))
                handler->queue_immediate_task(parent);
        }
    }
};

template <awaitable Awaitable>
using raw_await_result_t = decltype(std::declval<awaitable_awaiter_t<Awaitable>>().await_resume());

} // namespace detail

template <detail::awaitable... Awaitables>
    requires(sizeof...(Awaitables) > 1)
struct wait_for_all
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_for_all aw)
            : detail::awaiter(promise), awaitables(std::move(aw.awaitables))
        { std::ranges::fill(child_ids, detail::arena<task<>>::handle::null); }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        // NOLINTNEXTLINE(bugprone-exception-escape)
        ~awaiter_type()
        {
            for (auto id : child_ids)
                if (id != detail::arena<task<>>::handle::null) cancel_task(id);
        }

        template <typename Awaitable> detail::raw_await_result_t<Awaitable> default_for()
        { return std::unexpected(error{.value = error::unknown}); }

        std::tuple<detail::raw_await_result_t<Awaitables>...> results{default_for<Awaitables>()...};
        std::tuple<Awaitables...> awaitables;
        std::optional<detail::completion_barrier> barrier;
        std::array<detail::task_id, sizeof...(Awaitables)> child_ids;

        [[nodiscard]] bool await_ready() const noexcept { return false; }

        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            barrier.emplace(sizeof...(Awaitables), promise().id, &handler());
            spawn_children(std::index_sequence_for<Awaitables...>{});
        }
        auto on_resume() { return std::move(results); }

    private:
        template <std::size_t... Is> void spawn_children(std::index_sequence<Is...> /*unused*/)
        {
            ((child_ids[Is] =
                  add_task_eager(wrap_awaitable<Is>(std::get<Is>(std::move(awaitables))))),
             ...);
        }

        template <std::size_t I, typename Aw> task<> wrap_awaitable(Aw aw)
        {
            std::get<I>(results) = co_await std::move(aw);
            barrier->arrive();
        }
    };

    wait_for_all(Awaitables &&...awaitables) : awaitables(std::move(awaitables)...) {}

    std::tuple<Awaitables...> awaitables;
};

template <detail::awaitable... Awaitables>
    requires(sizeof...(Awaitables) > 1)
struct wait_for_any
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_for_any aw)
            : detail::awaiter(promise), awaitables(std::move(aw.awaitables))
        { std::ranges::fill(child_ids, detail::arena<task<>>::handle::null); }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        // NOLINTNEXTLINE(bugprone-exception-escape)
        ~awaiter_type()
        {
            for (auto id : child_ids)
                if (id != detail::arena<task<>>::handle::null) cancel_task(id);
        }

        std::variant<detail::raw_await_result_t<Awaitables>..., std::monostate> result =
            std::monostate{};
        std::tuple<Awaitables...> awaitables;
        std::array<detail::task_id, sizeof...(Awaitables)> child_ids;
        std::optional<detail::race_barrier> barrier;
        std::exception_ptr child_exception;

        [[nodiscard]] bool await_ready() const noexcept { return false; }

        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            barrier.emplace(false, 0, sizeof...(Awaitables), promise().id, &handler());
            spawn_children(std::index_sequence_for<Awaitables...>{});
        }

        auto on_resume()
        {
            for (auto &id : child_ids)
                cancel_task(std::exchange(id, detail::arena<task<>>::handle::null));

            if (child_exception && barrier->failures.load() == barrier->total)
                std::rethrow_exception(child_exception);
            return std::move(result);
        }

    private:
        template <std::size_t... Is> void spawn_children(std::index_sequence<Is...> /*unused*/)
        {
            ((child_ids[Is] =
                  add_task_eager(wrap_awaitable<Is>(std::get<Is>(std::move(awaitables))))),
             ...);
        }

        template <std::size_t I, typename Aw> task<> wrap_awaitable(Aw aw)
        {
            using await_result = detail::raw_await_result_t<Aw>;
            try
            {
                auto res = co_await std::move(aw);
                if (barrier->arrive()) result.template emplace<I>(std::move(res));
            }
            catch (...)
            {
                if (!child_exception) child_exception = std::current_exception();
                barrier->arrive_error();
            }
        }
    };

    wait_for_any(Awaitables &&...awaitables) : awaitables(std::move(awaitables)...) {}
    std::tuple<Awaitables...> awaitables;
};

namespace detail
{
template <typename T, typename... Args> struct command_awaiter : awaiter
{
    command_awaiter(task_promise_base &promise, Args... args, typename T::result_type &&init = {})
        : awaiter(promise), args(std::move(args)...), result(std::move(init))
    {
    }

    std::tuple<Args...> args;
    typename T::result_type result;

    [[nodiscard]] bool await_ready() const noexcept { return false; }
    void await_suspend(std::coroutine_handle<> handle)
    {
        world().push_command(std::apply([this, handle](Args &&...args)
                                        { return T(&result, handle, std::move(args)...); },
                                        std::move(args)));
    }

    auto on_resume() noexcept { return std::move(result); }
};
template <typename T, typename... Args> struct command_awaiter_no_wait : awaiter
{
    command_awaiter_no_wait(task_promise_base &promise, Args... args)
        : awaiter(promise), args(std::move(args)...)
    {
    }

    std::tuple<Args...> args;

    [[nodiscard]] bool await_ready() const noexcept { return true; }
    void await_suspend(std::coroutine_handle<> handle) const noexcept {}

    void on_resume() noexcept
    {
        world().push_command(std::apply([](Args &&...args)
                                        { return T(nullptr, nullptr, std::move(args)...); },
                                        std::move(args)));
    }
};

template <bool wait, typename T, typename... Args>
using select_command_awaiter =
    std::conditional_t<wait, command_awaiter<T, Args...>, command_awaiter_no_wait<T, Args...>>;
}; // namespace detail

// --------- COMMANDS ---------
enum class policy : std::uint8_t
{
    wait,
    no_wait,
};

template <policy p = policy::wait> struct add_task
{
    struct awaiter_type
        : detail::select_command_awaiter<(p == policy::wait), detail::add_task_command, task<>>
    {
        awaiter_type(detail::task_promise_base &promise, struct add_task aw)
            requires(p == policy::wait)
            : detail::select_command_awaiter<(p == policy::wait), detail::add_task_command, task<>>(
                  promise, std::move(aw.t),
                  detail::task_handler::task_handle::from_id(
                      detail::task_handler::task_handle::null))
        {
        }

        awaiter_type(detail::task_promise_base &promise, struct add_task aw)
            requires(p == policy::no_wait)
            : detail::select_command_awaiter<(p == policy::wait), detail::add_task_command, task<>>(
                  promise, std::move(aw.t))
        {
        }
    };
    task<> t;
};

template <policy p = policy::wait> struct cancel_task
{
    struct awaiter_type
        : detail::select_command_awaiter<(p == policy::wait), detail::cancel_task_command,
                                         detail::task_handler::task_handle>
    {
        awaiter_type(detail::task_promise_base &promise, struct cancel_task aw)
            : detail::select_command_awaiter<(p == policy::wait), detail::cancel_task_command,
                                             detail::task_handler::task_handle>(promise, aw.id)
        {
        }
    };
    detail::task_handler::task_handle id;
};

struct create_rigid
{
    struct awaiter_type : detail::command_awaiter<detail::add_rigid_command, object_desc>
    {
        awaiter_type(detail::task_promise_base &promise, create_rigid aw)
            : command_awaiter(promise, std::move(aw.desc),
                              object_handle::from_id(object_handle::null))
        {
        }
    };
    object_desc desc;
};

template <policy p = policy::wait> struct destroy_rigid
{
    struct awaiter_type
        : detail::select_command_awaiter<(p == policy::wait), detail::destroy_rigid_command,
                                         object_handle>
    {
        awaiter_type(detail::task_promise_base &promise, destroy_rigid aw)
            : detail::select_command_awaiter<(p == policy::wait), detail::destroy_rigid_command,
                                             object_handle>(promise, aw.h)
        {
        }

        auto on_resume()
            requires(p == policy::wait)
        {
            if (auto opt = std::move(this->result)) return std::move(*opt);
            throw error::stale_handle;
        }
    };
    object_handle h;
};

struct get_rigid
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, get_rigid aw)
            : awaiter(promise), handle(aw.h)
        {
        }

        object_handle handle;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() const noexcept { return true; }
        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        bool await_suspend(std::coroutine_handle<> handle) { return false; }
        auto on_resume()
        {
            if (auto r = world().get_rigid(handle)) return *r;
            throw error::stale_handle;
        }
    };
    object_handle h;
};

} // namespace physkit