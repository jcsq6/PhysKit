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
        void await_suspend(std::coroutine_handle<> /**/) { handler().queue_task(promise().id); }
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
            : detail::awaiter(promise), object(aw.object)
        {
        }

        world_base::handle object;
        detail::narrow_phase::manifold_info result;

        [[nodiscard]] bool await_ready() const { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<> /*handle*/)
        { handler().add_collision_waiter(object.id(), promise().id, &result); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        result_type on_resume() const noexcept
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

struct wait_for_separation
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, wait_for_separation aw)
            : detail::awaiter(promise), object(aw.object)
        {
        }

        world_base::handle object;
        detail::handle_id_t result_a = world_base::handle::null;
        detail::handle_id_t result_b = world_base::handle::null;

        // TODO: maybe make sure it's actually colliding?
        [[nodiscard]] bool await_ready() const { return !world().get_rigid(object); }
        void await_suspend(std::coroutine_handle<> /*handle*/)
        { handler().add_collision_exit_waiter(object.id(), promise().id, &result_a, &result_b); }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        world_base::handle on_resume() const noexcept
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
        if (--remaining == 0) handler->queue_task_immediate(parent);
    }
};

struct race_barrier
{
    std::atomic<bool> finished{false};
    task_id parent;
    task_handler *handler;

    bool arrive()
    {
        bool expected = false;
        if (finished.compare_exchange_strong(expected, true))
        {
            handler->queue_task_immediate(parent);
            return true;
        }
        return false;
    }
};

template <awaitable Awaitable>
using awaitable_result_t = std::conditional_t<
    std::is_void_v<decltype(std::declval<awaitable_awaiter_t<Awaitable>>().await_resume())>,
    std::monostate, decltype(std::declval<awaitable_awaiter_t<Awaitable>>().await_resume())>;

} // namespace detail

template <detail::awaitable... Awaitables>
    requires(sizeof...(Awaitables) > 1)
struct after_all
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, after_all aw)
            : detail::awaiter(promise), awaitables(std::move(aw.awaitables))
        { std::ranges::fill(child_ids, detail::arena<task<>>::handle::null); }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        ~awaiter_type()
        {
            for (auto id : child_ids)
                if (id != detail::arena<task<>>::handle::null) cancel_task(id);
        }

        std::tuple<detail::awaitable_result_t<Awaitables>...> results;
        std::tuple<Awaitables...> awaitables;
        std::optional<detail::completion_barrier> barrier;
        std::array<detail::task_id, sizeof...(Awaitables)> child_ids;

        [[nodiscard]] bool await_ready() const noexcept { return false; }

        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            barrier.emplace(sizeof...(Awaitables), promise().id, &handler());
            spawn_children(std::index_sequence_for<Awaitables...>{});
        }
        // NOLINTNEXTLINE(modernize-use-nodiscard)
        auto on_resume() const noexcept { return std::move(results); }

    private:
        template <std::size_t... Is> void spawn_children(std::index_sequence<Is...> /*unused*/)
        {
            ((child_ids[Is] =
                  add_task_eager(wrap_awaitable<Is>(std::get<Is>(std::move(awaitables))))),
             ...);
        }

        template <std::size_t I, typename Aw> task<> wrap_awaitable(Aw aw)
        {
            if constexpr (std::is_void_v<decltype(std::declval<detail::awaitable_awaiter_t<Aw>>()
                                                      .await_resume())>)
                co_await std::move(aw);
            else
                std::get<I>(results) = co_await std::move(aw);
            barrier->arrive();
        }
    };

    after_all(Awaitables &&...awaitables) : awaitables(std::move(awaitables)...) {}

    std::tuple<Awaitables...> awaitables;
};

template <detail::awaitable... Awaitables>
    requires(sizeof...(Awaitables) > 1)
struct after_any
{
    struct awaiter_type : detail::awaiter
    {
        awaiter_type(detail::task_promise_base &promise, after_any aw)
            : detail::awaiter(promise), awaitables(std::move(aw.awaitables))
        { std::ranges::fill(child_ids, detail::arena<task<>>::handle::null); }

        awaiter_type(const awaiter_type &) = delete;
        awaiter_type &operator=(const awaiter_type &) = delete;
        awaiter_type(awaiter_type &&) = delete;
        awaiter_type &operator=(awaiter_type &&) = delete;

        ~awaiter_type()
        {
            for (auto id : child_ids)
                if (id != detail::arena<task<>>::handle::null) cancel_task(id);
        }

        std::variant<detail::awaitable_result_t<Awaitables>...> result;
        std::tuple<Awaitables...> awaitables;
        std::array<detail::task_id, sizeof...(Awaitables)> child_ids;
        std::optional<detail::race_barrier> barrier;

        [[nodiscard]] bool await_ready() const noexcept { return false; }

        void await_suspend(std::coroutine_handle<> /*handle*/)
        {
            barrier.emplace(false, promise().id, &handler());
            spawn_children(std::index_sequence_for<Awaitables...>{});
        }

        auto on_resume()
        {
            for (auto id : child_ids)
                if (id != detail::arena<task<>>::handle::null) cancel_task(id);
            return std::move(result);
        }

    private:
        template <std::size_t... Is> void spawn_children(std::index_sequence<Is...> /*unused*/)
        {
            (((child_ids[Is] =
                   add_task_eager(wrap_awaitable<Is>(std::get<Is>(std::move(awaitables))))),
              ...));
        }

        template <std::size_t I, typename Aw> task<> wrap_awaitable(Aw aw)
        {
            if constexpr (std::is_void_v<decltype(std::declval<detail::awaitable_awaiter_t<Aw>>()
                                                      .await_resume())>)
            {
                co_await std::move<Aw>(aw);
                if (barrier->arrive()) result.template emplace<I>(std::monostate{});
            }
            else
            {
                auto res = co_await std::move(aw);
                if (barrier->arrive()) result.template emplace<I>(std::move(res));
            }
        }
    };

    after_any(Awaitables &&...awaitables) : awaitables(std::move(awaitables)...) {}
    std::tuple<Awaitables...> awaitables;
};

} // namespace physkit