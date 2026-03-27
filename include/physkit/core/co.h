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

template <typename T = void> class task;

namespace detail
{
using task_id = arena<task<>>::handle::id_type;

struct task_promise_base;
template <typename T> class task_promise;
template <typename T> class task_awaiter;
class task_handler;

class awaiter
{
public:
    awaiter(task_promise_base &promise) : M_promise(promise) {}

    auto await_resume(this auto &&self)
    {
        self.promise().check_rethrow();
        return self.on_resume();
    }

protected:
    [[nodiscard]] task_promise_base &promise() const { return M_promise; }
    [[nodiscard]] task_handler &handler() const;
    [[nodiscard]] world_base &world() const;

    task_id add_task(task<> t);
    task_id add_task_eager(task<> t);
    void cancel_task(task_id id);

    [[nodiscard]] bool has_world() const;

private:
    // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
    task_promise_base &M_promise;
};

template <typename Awaitable> using awaitable_awaiter_t = Awaitable::awaiter_type;
template <typename Awaiter>
concept awaiter_t = requires(Awaiter aw, std::coroutine_handle<task_promise_base> handle) {
    { aw.await_ready() } -> std::convertible_to<bool>;
    { aw.await_suspend(handle) };
    { aw.await_resume() };
} && (!std::derived_from<Awaiter, detail::awaiter> || requires(Awaiter aw) {
                        { aw.on_resume() };
                    });

template <typename Awaitable>
concept awaitable = requires(task_promise_base &p, Awaitable &&aw) {
    requires awaiter_t<awaitable_awaiter_t<Awaitable>>;
    awaitable_awaiter_t<Awaitable>(p, std::forward<Awaitable>(aw));
};

struct task_promise_base
{
    struct final_awaiter
    {
        // NOLINTNEXTLINE(cppcoreguidelines-avoid-const-or-ref-data-members)
        task_promise_base &promise;

        // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
        [[nodiscard]] bool await_ready() const noexcept { return false; }
        [[nodiscard]] std::coroutine_handle<>
        await_suspend(std::coroutine_handle<> handle) const noexcept
        {
            if (promise.continuation)
            {
                *promise.root_leaf_ptr = promise.continuation;
                return promise.continuation;
            }
            return std::noop_coroutine();
        }
        void await_resume() noexcept {}
    };

    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    std::suspend_always initial_suspend() noexcept { return {}; }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    final_awaiter final_suspend() noexcept { return {*this}; }
    // NOLINTNEXTLINE(readability-convert-member-functions-to-static)
    void unhandled_exception() { exception = std::current_exception(); }

    template <awaitable Awaitable>
    detail::awaitable_awaiter_t<Awaitable> await_transform(Awaitable &&request)
    { return {*this, std::forward<Awaitable>(request)}; }

    auto await_transform(auto &&other) = delete;

    void check_rethrow() const
    {
        if (exception) std::rethrow_exception(exception);
    }

    world_base *world{};
    task_id id = detail::arena<task_id>::handle::null;

    std::coroutine_handle<> root_leaf;
    std::coroutine_handle<> *root_leaf_ptr = &root_leaf;
    std::coroutine_handle<> continuation;
    std::exception_ptr exception;
};

inline bool awaiter::has_world() const { return M_promise.world != nullptr; }
inline world_base &awaiter::world() const { return *M_promise.world; }

template <typename T> struct task_promise : public task_promise_base
{
    task<T> get_return_object();
    void return_value(T value) { result.emplace(std::move(value)); }
    T get_result() { return std::move(*result); }
    std::optional<T> result;
};

template <> struct task_promise<void> : public task_promise_base
{
    task<void> get_return_object();
    void return_void() noexcept {}
    void get_result() const {}
};

} // namespace detail

template <typename T> class task
{
public:
    using promise_type = detail::task_promise<T>;
    using awaiter_type = detail::task_awaiter<T>;

    explicit task(std::coroutine_handle<promise_type> h) : M_handle(h)
    {
        if (M_handle) M_handle.promise().root_leaf = M_handle;
    }
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
            auto leaf = *M_handle.promise().root_leaf_ptr;
            if (leaf && !leaf.done()) leaf.resume();

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
    [[nodiscard]] detail::task_promise_base &promise_base() const { return M_handle.promise(); }
    T get_result() const { return M_handle.promise().get_result(); }

    std::coroutine_handle<promise_type> M_handle;

    template <typename U> friend struct detail::task_awaiter;
};

namespace detail
{

template <typename T> struct task_awaiter
{
    task_awaiter(detail::task_promise_base & /*unused*/, task<T> t) : t(std::move(t)) {}

    task<T> t;

    [[nodiscard]] bool await_ready() const noexcept { return t.done(); }

    template <typename Promise>
    std::coroutine_handle<> await_suspend(std::coroutine_handle<Promise> caller) noexcept
    {
        auto &child_promise = t.promise_base();
        auto &parent_promise = caller.promise();

        child_promise.world = parent_promise.world;
        child_promise.id = parent_promise.id;

        child_promise.root_leaf_ptr = parent_promise.root_leaf_ptr;
        child_promise.continuation = caller;

        *child_promise.root_leaf_ptr = t.M_handle;

        return t.M_handle;
    }

    T await_resume() const
    {
        t.promise_base().check_rethrow();
        return t.get_result();
    }
};

template <typename T> task<T> task_promise<T>::get_return_object()
{ return task<T>{std::coroutine_handle<task_promise<T>>::from_promise(*this)}; }

inline task<void> task_promise<void>::get_return_object()
{ return task<void>{std::coroutine_handle<task_promise<void>>::from_promise(*this)}; }

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
    void queue_task(const task_id id) { M_pending_tasks.push(id); }

    // Queue task for immediate execution within the current process_tasks call
    void queue_task_immediate(const task_id id) { M_immediate_tasks.push(id); }

    [[nodiscard]] auto time() const { return M_current_time; }

    void increment(const mp_units::quantity<mp_units::si::second> dt, passkey<world_base> /*key*/)
    { M_current_time += dt; }

    task_id add_task(task<> t, world_base &world, passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = M_tasks.add(std::move(t));
        auto id = handle.id();

        auto *allocated_task = M_tasks.get(handle);
        allocated_task->set_id(id, {});
        allocated_task->set_world(world, {});

        M_pending_tasks.push(id);
        return id;
    }

    // Add and immediately start a task (runs to its first suspension point)
    task_id add_task_eager(task<> t, world_base &world,
                           passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = M_tasks.add(std::move(t));
        auto id = handle.id();

        auto *allocated_task = M_tasks.get(handle);
        allocated_task->set_id(id, {});
        allocated_task->set_world(world, {});

        if (allocated_task->resume({})) M_tasks.remove(handle);

        return id;
    }

    void cancel_task(const task_id id, passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = detail::arena<task<>>::handle::from_id(id);
        M_tasks.remove(handle);
    }

    void process_tasks(passkey<world_base> /*key*/)
    {
        M_pending_tasks.swap_and_clear();

        drain_immediate();

        for (auto id : M_pending_tasks.swap) resume_task(id);

        while (auto t = next_task()) resume_task(*t);

        drain_immediate();
    }

    void on_collision(const narrow_phase::manifold_info &man_info, handle_id_t obj_a,
                      handle_id_t obj_b)
    {
        auto on_obj = [&](handle_id_t object_id)
        {
            auto it = M_object_waiters.find(object_id);
            if (it == M_object_waiters.end()) return;
            for (auto [tid, result_storage] : it->second.coll_enter)
            {
                if (!M_tasks.get(detail::arena<task<>>::handle::from_id(tid))) continue;
                if (result_storage) *result_storage = man_info;
                queue_task_immediate(tid);
            }
            it->second.coll_enter.clear();
            if (it->second.coll_exit.empty()) M_object_waiters.erase(it);
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
            for (auto [tid, a, b] : it->second.coll_exit)
            {
                // TODO: clean up stale waiters before this
                if (!M_tasks.get(detail::arena<task<>>::handle::from_id(tid))) continue;
                if (a) *a = obj_a;
                if (b) *b = obj_b;
                queue_task_immediate(tid);
            }
            it->second.coll_exit.clear();
            if (it->second.coll_enter.empty()) M_object_waiters.erase(it);
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

    struct swappable_queue
    {
        std::vector<handle_id_t> main;
        std::vector<handle_id_t> swap;

        void push(handle_id_t id) { main.push_back(id); }
        void swap_and_clear()
        {
            swap.clear();
            swap.swap(main);
        }
    };

    detail::arena<task<>> M_tasks;
    swappable_queue M_pending_tasks;
    swappable_queue M_immediate_tasks;
    absl::flat_hash_map<handle_id_t, object_waiter> M_object_waiters;

    std::vector<std::pair<mp_units::quantity<mp_units::si::second>, handle_id_t>>
        M_timer_heap; // TODO: memory management
    mp_units::quantity<mp_units::si::second> M_current_time{0.0 * mp_units::si::second};

    void drain_immediate()
    {
        while (!M_immediate_tasks.main.empty())
        {
            M_immediate_tasks.swap_and_clear();
            for (auto id : M_immediate_tasks.swap) resume_task(id);
        }
    }

    void resume_task(handle_id_t id)
    {
        auto handle = detail::arena<task<>>::handle::from_id(id);
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

inline task_id awaiter::add_task(task<> t) { return handler().add_task(std::move(t), world(), {}); }
inline task_id awaiter::add_task_eager(task<> t)
{ return handler().add_task_eager(std::move(t), world(), {}); }
inline void awaiter::cancel_task(task_id id) { handler().cancel_task(id, {}); }
} // namespace detail
} // namespace physkit