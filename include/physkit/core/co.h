#pragma once
#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "../detail/arena.h"
#include "../detail/macros.h"
#include "../detail/util.h"
#include <cassert>

#include <algorithm>
#include <coroutine>
#include <vector>

#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>
#endif

PHYSKIT_EXPORT
namespace physkit
{

class world_base;
class task;

namespace detail
{
using task_id = arena<task>::handle::id_type;

class task_scheduler
{
public:
    struct task
    {
        task(task_id id, mp_units::quantity<mp_units::si::second> when) : M_id(id), M_when(when) {}

        [[nodiscard]] auto when() const { return M_when; }
        [[nodiscard]] auto id() const { return M_id; }

        auto operator<=>(const task &o) const { return M_when <=> o.M_when; }
        auto operator==(const task &o) const { return M_when == o.M_when; }

    private:
        task_id M_id;
        mp_units::quantity<mp_units::si::second> M_when{0.0 * mp_units::si::second};
    };

    void at(task_id id, mp_units::quantity<mp_units::si::second> when)
    {
        M_timer_heap.emplace_back(id, when);
        std::ranges::push_heap(M_timer_heap, std::ranges::greater{});
    }

    void after(task_id id, mp_units::quantity<mp_units::si::second> delay)
    { at(id, M_current_time + delay); }

    std::optional<task> next_task()
    {
        if (M_timer_heap.empty() || M_timer_heap.front().when() > M_current_time)
            return std::nullopt;
        auto t = M_timer_heap.front();
        std::ranges::pop_heap(M_timer_heap, std::ranges::greater{});
        M_timer_heap.pop_back();
        return t;
    }

    void increment(mp_units::quantity<mp_units::si::second> dt) { M_current_time += dt; }

    [[nodiscard]] auto time() const { return M_current_time; }

private:
    std::vector<task> M_timer_heap; // TODO: memory management
    mp_units::quantity<mp_units::si::second> M_current_time{0.0 * mp_units::si::second};
};

struct task_promise;

class awaiter
{
public:
    awaiter(task_promise &promise) : M_promise(promise) {}

protected:
    task_promise &promise(this auto &&self)
    { return std::forward_like<decltype(self)>(self.M_promise); }
    [[nodiscard]] world_base &world() const;

    [[nodiscard]] task_scheduler &scheduler() const;

    [[nodiscard]] bool has_world() const;

    void queue_task(task_id id) const;

private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    task_promise &M_promise;
};

template <typename Awaitable> using awaitable_awaiter_t = typename Awaitable::awaiter_type;
template <typename Awaiter>
concept awaiter_t =
    std::derived_from<Awaiter, awaiter> &&
    requires(task_promise p, Awaiter aw, std::coroutine_handle<task_promise> handle) {
        { aw.await_ready() } -> std::convertible_to<bool>;
        { aw.await_suspend(handle) };
        { aw.await_resume() };
    };

template <typename Awaitable>
concept awaitable = requires(task_promise p, Awaitable &&aw) {
    awaiter_t<awaitable_awaiter_t<Awaitable>>;
    awaitable_awaiter_t<Awaitable>(p, std::forward<Awaitable>(aw));
};

struct task_promise
{
    // TODO: operator new()

    task get_return_object();
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    std::suspend_always initial_suspend() noexcept { return {}; }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    std::suspend_always final_suspend() noexcept { return {}; }
    void return_void() noexcept {}
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void unhandled_exception() { throw; } // TODO: change on multithread support

    template <awaitable Awaitable>
    detail::awaitable_awaiter_t<Awaitable> await_transform(Awaitable &&request)
    { return {*this, std::forward<Awaitable>(request)}; }

    auto await_transform(auto &&other) = delete;

    world_base *world{};
    task_id id = detail::arena<task_id>::handle::null;
};

inline world_base &awaiter::world() const
{
    assert(M_promise.world != nullptr && "Promise must have a valid world pointer");
    return *M_promise.world;
}

inline bool awaiter::has_world() const { return M_promise.world != nullptr; }

} // namespace detail

class task
{
public:
    using promise_type = detail::task_promise;

    explicit task(std::coroutine_handle<promise_type> h) : M_handle(h) {}
    task(const task &) = delete;
    task(task &&other) noexcept : M_handle(other.M_handle) { other.M_handle = nullptr; }
    task &operator=(const task &) = delete;
    task &operator=(task &&other) noexcept
    {
        if (this != &other)
        {
            if (M_handle) M_handle.destroy();
            M_handle = other.M_handle;
            other.M_handle = nullptr;
        }
        return *this;
    }
    ~task()
    {
        if (M_handle) M_handle.destroy();
    }

    void set_world(world_base &w, detail::passkey<world_base> /*key*/) const
    { M_handle.promise().world = &w; }
    void set_id(detail::task_id id, detail::passkey<world_base> /*key*/) const
    { M_handle.promise().id = id; }
    [[nodiscard]] bool done() const { return !M_handle || M_handle.done(); }

    bool resume(detail::passkey<world_base> /*key*/)
    {
        if (M_handle)
        {
            M_handle.resume();
            if (M_handle.done())
            {
                M_handle.destroy();
                M_handle = nullptr;
                return true;
            }
        }

        return false;
    }

    [[nodiscard]] auto handle(detail::passkey<world_base> /*key*/) const { return M_handle; }

    // NOLINTNEXTLINE(readability-identifier-naming)
    static auto _passkey(detail::passkey<detail::awaiter> /*key*/)
    { return detail::passkey<task>{}; }

private:
    std::coroutine_handle<promise_type> M_handle;
};

inline task detail::task_promise::get_return_object()
{ return task{std::coroutine_handle<detail::task_promise>::from_promise(*this)}; }
} // namespace physkit