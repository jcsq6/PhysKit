#pragma once
#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "../collision/collision_phases.h"
#include "../detail/arena.h"
#include "../detail/macros.h"

#include <cassert>

#include <algorithm>
#include <coroutine>
#include <vector>

#include <absl/container/flat_hash_map.h>

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

struct task_promise;
class task_handler;

class awaiter
{
public:
    awaiter(task_promise &promise) : M_promise(promise) {}

protected:
    [[nodiscard]] task_promise &promise() const { return M_promise; }
    [[nodiscard]] task_handler &handler() const;
    [[nodiscard]] world_base &world() const;

    [[nodiscard]] bool has_world() const;

private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    task_promise &M_promise;
};

template <typename Awaitable> using awaitable_awaiter_t = Awaitable::awaiter_type;
template <typename Awaiter>
concept awaiter_t = std::derived_from<Awaiter, awaiter> &&
                    requires(Awaiter aw, std::coroutine_handle<task_promise> handle) {
                        { aw.await_ready() } -> std::convertible_to<bool>;
                        { aw.await_suspend(handle) };
                        { aw.await_resume() };
                    };

template <typename Awaitable>
concept awaitable = requires(task_promise &p, Awaitable &&aw) {
    requires awaiter_t<awaitable_awaiter_t<Awaitable>>;
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
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
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

inline bool awaiter::has_world() const { return M_promise.world != nullptr; }
inline world_base &awaiter::world() const { return *M_promise.world; }

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

    void set_world(world_base &w, detail::passkey<detail::task_handler> /*key*/) const
    { M_handle.promise().world = &w; }
    void set_id(detail::handle_id_t id, detail::passkey<detail::task_handler> /*key*/) const
    { M_handle.promise().id = id; }
    [[nodiscard]] bool done() const { return !M_handle || M_handle.done(); }

    bool resume(detail::passkey<detail::task_handler> /*key*/)
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

    [[nodiscard]] auto handle(detail::passkey<detail::task_handler> /*key*/) const
    { return M_handle; }

private:
    std::coroutine_handle<promise_type> M_handle;
};

inline task detail::task_promise::get_return_object()
{ return task{std::coroutine_handle<detail::task_promise>::from_promise(*this)}; }

namespace detail
{

class task_handler
{
public:
    void schedule_task_at(const task_id id, const mp_units::quantity<mp_units::si::second> when)
    {
        M_timer_heap.emplace_back(when, id);
        std::ranges::push_heap(M_timer_heap,
                               [](const auto &a, const auto &b) { return a.first > b.first; });
    }

    void schedule_task_after(const task_id id, const mp_units::quantity<mp_units::si::second> delay)
    { schedule_task_at(id, M_current_time + delay); }

    // Queue task for execution in the next frame
    void queue_task(const handle_id_t task_id) { M_pending_tasks.push_back(task_id); }

    [[nodiscard]] auto time() const { return M_current_time; }

    void increment(const mp_units::quantity<mp_units::si::second> dt, passkey<world_base> /*key*/)
    { M_current_time += dt; }

    void add_task(task t, world_base &world, passkey<world_base> /*key*/)
    {
        auto handle = M_tasks.add(std::move(t));
        auto id = handle.id();

        auto *allocated_task = M_tasks.get(handle);
        allocated_task->set_id(id, {});
        allocated_task->set_world(world, {});

        M_pending_tasks.push_back(id);
    }

    void process_tasks(passkey<world_base> /*key*/)
    {
        for (auto id : M_pending_tasks) resume_task(id);
        M_pending_tasks.clear();

        while (auto t = next_task()) resume_task(*t);
    }

    void on_collision(const narrow_phase::manifold_info &man_info, handle_id_t obj_a,
                      handle_id_t obj_b)
    {
        auto on_obj = [&](handle_id_t object_id)
        {
            auto it = M_object_waiters.find(object_id);
            if (it == M_object_waiters.end()) return;
            for (auto [task, result_storage] : it->second.coll_enter)
            {
                if (result_storage) *result_storage = man_info;
                queue_task(task);
            }
        };

        on_obj(obj_a);
        on_obj(obj_b);
    }

    void on_collision_exit(handle_id_t obj_a, handle_id_t obj_b)
    {
        auto on_obj = [&](handle_id_t object_id)
        {
            auto it = M_object_waiters.find(object_id);
            if (it == M_object_waiters.end()) return;
            for (auto [task, a, b] : it->second.coll_exit)
            {
                if (a) *a = obj_a;
                if (b) *b = obj_b;
                queue_task(task);
            }
        };

        on_obj(obj_a);
        on_obj(obj_b);
    }

    void add_collision_waiter(handle_id_t object_id, handle_id_t task_id,
                              narrow_phase::manifold_info *result_storage)
    {
        M_object_waiters[object_id].coll_enter.push_back(
            {.task_id = task_id, .result_storage = result_storage});
    }

    void add_collision_exit_waiter(handle_id_t object_id, handle_id_t task_id, handle_id_t *a,
                                   handle_id_t *b)
    { M_object_waiters[object_id].coll_exit.push_back({.task_id = task_id, .a = a, .b = b}); }

private:
    struct object_waiter
    {
        struct collision_enter_res
        {
            handle_id_t task_id;
            narrow_phase::manifold_info *result_storage;
        };
        struct collision_exit_res
        {
            handle_id_t task_id;
            handle_id_t *a;
            handle_id_t *b;
        };
        std::vector<collision_enter_res> coll_enter;
        std::vector<collision_exit_res> coll_exit;
    };

    detail::arena<task> M_tasks;
    std::vector<handle_id_t> M_pending_tasks;
    absl::flat_hash_map<handle_id_t, object_waiter> M_object_waiters;

    std::vector<std::pair<mp_units::quantity<mp_units::si::second>, handle_id_t>>
        M_timer_heap; // TODO: memory management
    mp_units::quantity<mp_units::si::second> M_current_time{0.0 * mp_units::si::second};

    void resume_task(handle_id_t id)
    {
        auto handle = detail::arena<task>::handle::from_id(id);
        if (auto *t = M_tasks.get(handle))
            if (t->resume({})) M_tasks.remove(handle);
    }

    std::optional<handle_id_t> next_task()
    {
        if (M_timer_heap.empty() || M_timer_heap.front().first > M_current_time)
            return std::nullopt;
        auto t = M_timer_heap.front();
        std::ranges::pop_heap(M_timer_heap,
                              [](const auto &a, const auto &b) { return a.first > b.first; });
        M_timer_heap.pop_back();
        return t.second;
    }
};
} // namespace detail
} // namespace physkit