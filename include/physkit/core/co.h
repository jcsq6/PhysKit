#pragma once
#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include "../collision/collision_phases.h"
#include "../detail/arena.h"
#include "../detail/macros.h"
#include "../detail/swappable_queue.h"

#include <cassert>

#include <algorithm>
#include <coroutine>
#include <expected>
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

struct error
{
    enum code : std::uint8_t
    {
        stale_handle,
        object_destroyed,
        exception_thrown,
        unknown,
    };
    code value;
    std::exception_ptr exception;
};

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

    auto await_resume(this auto &&self) -> std::expected<decltype(self.on_resume()), error>
    {
        try
        {
            self.promise().check_rethrow();
            if constexpr (std::is_void_v<decltype(self.on_resume())>)
            {
                self.on_resume();
                return {};
            }
            else
            {
                return self.on_resume();
            }
        }
        catch (error::code c)
        {
            return std::unexpected(error{.value = c});
        }
        catch (...)
        {
            return std::unexpected(
                error{.value = error::exception_thrown, .exception = std::current_exception()});
        }
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

    void set_error(auto &&e) { exception = std::make_exception_ptr(std::forward<decltype(e)>(e)); }

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
    void set_error(auto &&e, detail::passkey<detail::task_handler> /*key*/)
    { M_handle.promise().set_error(std::forward<decltype(e)>(e)); }

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

    auto await_resume() const -> std::expected<T, error>
    {
        try
        {
            t.promise_base().check_rethrow();
            if constexpr (std::is_void_v<T>)
            {
                t.get_result();
                return {};
            }
            else
                return t.get_result();
        }
        catch (error::code c)
        {
            return std::unexpected(error{.value = c});
        }
        catch (...)
        {
            return std::unexpected(
                error{.value = error::exception_thrown, .exception = std::current_exception()});
        }
    }
};

template <typename T> task<T> task_promise<T>::get_return_object()
{ return task<T>{std::coroutine_handle<task_promise<T>>::from_promise(*this)}; }

inline task<void> task_promise<void>::get_return_object()
{ return task<void>{std::coroutine_handle<task_promise<void>>::from_promise(*this)}; }

class task_handler
{
public:
    using task_handle = arena<task<>>::handle;

    void schedule_task_at(const task_id id, const mp_units::quantity<mp_units::si::second> when)
    {
        M_timer_heap.emplace_back(when, id);
        std::ranges::push_heap(M_timer_heap,
                               [](const auto &a, const auto &b) { return a.first > b.first; });
    }

    void schedule_task_after(const task_id id, const mp_units::quantity<mp_units::si::second> delay)
    { schedule_task_at(id, M_current_time + delay); }

    // Queue task for execution in beginning of the frame, before physics
    void queue_pre_task(const task_id id) { M_pre_queue.push(id); }

    // Queue task for execution after physics are calculated
    void queue_post_task(const task_id id) { M_post_queue.push(id); }

    // Queue task for execution after physics are calculated and after post_tasks are run
    void queue_post_physics_tick(const task_id id) { M_post_physics_queue.push(id); }

    // Queue for cleanup tasks run at the very end of the frame
    void queue_cleanup_task(const task_id id) { M_cleanup_queue.push(id); }

    // Queue for tasks that should be run immediately after the current phase
    void queue_immediate_task(const task_id id) { M_immediate_queue.push(id); }

    [[nodiscard]] auto time() const { return M_current_time; }

    void increment(const mp_units::quantity<mp_units::si::second> dt, passkey<world_base> /*key*/)
    { M_current_time += dt; }

    task_handle add_task(task<> t, world_base &world, passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = M_tasks.add(std::move(t));
        auto id = handle.id();

        auto *allocated_task = M_tasks.get(handle);
        allocated_task->set_id(id, {});
        allocated_task->set_world(world, {});

        M_pre_queue.push(id);
        return handle;
    }

    // Add and immediately start a task (runs to its first suspension point)
    task_handle add_task_eager(task<> t, world_base &world,
                               passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = M_tasks.add(std::move(t));
        auto id = handle.id();

        auto *allocated_task = M_tasks.get(handle);
        allocated_task->set_id(id, {});
        allocated_task->set_world(world, {});

        if (allocated_task->resume({})) M_tasks.remove(handle);

        return handle;
    }

    void cancel_task(const task_id id, passkey<world_base, detail::awaiter> /*key*/)
    {
        auto handle = detail::arena<task<>>::handle::from_id(id);
        M_tasks.remove(handle);
    }

    void dispatch_pre(passkey<world_base> /*key*/)
    {
        for (auto id : M_pre_queue.iter()) resume_task(id);
        while (auto t = next_task()) resume_task(*t);

        drain(M_immediate_queue);
    }

    void dispatch_post(passkey<world_base> /*key*/)
    {
        drain(M_post_queue);
        drain(M_immediate_queue);
        for (auto id : M_post_physics_queue.iter()) resume_task(id);
        drain(M_immediate_queue);
    }

    void dispatch_cleanup(std::regular_invocable<> auto &&flush_commands,
                          passkey<world_base> /*key*/)
        requires(std::same_as<bool, std::invoke_result_t<decltype(flush_commands)>>)
    {
        bool done = false;
        while (!done)
        {
            done = !drain(M_cleanup_queue);
            done = !drain(M_immediate_queue) && done;
            done = !flush_commands() && done;
        }
    }

    void on_collision(const narrow_phase::manifold_info &man_info, handle_id_t obj_a,
                      handle_id_t obj_b, passkey<world_base> /*key*/)
    {
        auto on_obj = [&](handle_id_t object_id, handle_id_t other_id)
        {
            auto it = M_object_waiters.find(object_id);
            if (it == M_object_waiters.end()) return;
            std::erase_if(it->second.coll_enter,
                          [&](const auto &w)
                          {
                              if (w.with != other_id && w.with != null_id) return false;

                              if (w.result_storage) *w.result_storage = man_info;
                              queue_post_task(w.task_id);
                              return true;
                          });
            if (it->second.empty()) M_object_waiters.erase(it);
        };

        on_obj(obj_a, obj_b);
        on_obj(obj_b, obj_a);
    }

    void on_collision_exit(handle_id_t obj_a, handle_id_t obj_b, passkey<world_base> /*key*/)
    {
        auto on_obj = [&](handle_id_t object_id, handle_id_t other_id)
        {
            auto it = M_object_waiters.find(object_id);
            if (it == M_object_waiters.end()) return;
            std::erase_if(it->second.coll_exit,
                          [&](const auto &w)
                          {
                              if (w.with != other_id && w.with != null_id) return false;

                              if (w.a) *w.a = obj_a;
                              if (w.b) *w.b = obj_b;

                              queue_post_task(w.task_id);
                              return true;
                          });
            if (it->second.empty()) M_object_waiters.erase(it);
        };

        on_obj(obj_a, obj_b);
        on_obj(obj_b, obj_a);
    }

    void on_destruction(handle_id_t obj, passkey<world_base> /*key*/)
    {

        if (auto it = M_object_waiters.find(obj); it != M_object_waiters.end())
        {
            for (auto [tid] : it->second.destructions) queue_cleanup_task(tid);
            for (auto [tid, with, a, b] : it->second.coll_exit)
            {
                if (with != null_id) remove_dependency(with, tid);
                if (auto *task = M_tasks.get(task_handle::from_id(tid)))
                {
                    task->set_error(error::object_destroyed, {});
                    queue_cleanup_task(tid);
                }
            }
            for (auto [tid, with, result_storage] : it->second.coll_enter)
            {
                if (with != null_id) remove_dependency(with, tid);
                if (auto *task = M_tasks.get(task_handle::from_id(tid)))
                {
                    task->set_error(error::object_destroyed, {});
                    queue_cleanup_task(tid);
                }
            }
            M_object_waiters.erase(it);
        }

        if (auto it = M_object_deps.find(obj); it != M_object_deps.end())
        {
            thread_local std::vector<dependency> copy;
            copy = it->second;
            for (auto [tid, source_obj] : copy)
            {
                remove_collision_waiter(source_obj, tid);
                remove_collision_exit_waiter(source_obj, tid);
                if (auto *task = M_tasks.get(task_handle::from_id(tid)))
                {
                    task->set_error(error::object_destroyed, {});
                    queue_cleanup_task(tid);
                }
            }
            M_object_deps.erase(it);
        }
    }

    void add_collision_waiter(handle_id_t object_id, handle_id_t with, handle_id_t task_id,
                              narrow_phase::manifold_info *result_storage)
    {
        M_object_waiters[object_id].coll_enter.emplace_back(task_id, with, result_storage);
        if (with != null_id) M_object_deps[with].emplace_back(task_id, object_id);
    }

    void add_collision_exit_waiter(handle_id_t object_id, handle_id_t with, handle_id_t task_id,
                                   handle_id_t *a, handle_id_t *b)
    {
        M_object_waiters[object_id].coll_exit.emplace_back(task_id, with, a, b);
        if (with != null_id) M_object_deps[with].emplace_back(task_id, object_id);
    }

    void add_destruction_waiter(handle_id_t object_id, handle_id_t task_id)
    { M_object_waiters[object_id].destructions.emplace_back(task_id); }

    void remove_collision_waiter(handle_id_t object_id, handle_id_t task_id)
    { remove_waiter<object_waiter::type::collision_enter>(object_id, task_id); }

    void remove_collision_exit_waiter(handle_id_t object_id, handle_id_t task_id)
    { remove_waiter<object_waiter::type::collision_exit>(object_id, task_id); }

    void remove_destruction_waiter(handle_id_t object_id, handle_id_t task_id)
    { remove_waiter<object_waiter::type::destruction>(object_id, task_id); }

private:
    struct object_waiter
    {
        enum class type : std::uint8_t
        {
            collision_enter,
            collision_exit,
            destruction
        };

        struct collision_enter_res
        {
            task_id task_id;
            handle_id_t with;
            narrow_phase::manifold_info *result_storage;
        };
        struct collision_exit_res
        {
            task_id task_id;
            handle_id_t with;
            handle_id_t *a;
            handle_id_t *b;
        };
        struct destruction_res
        {
            task_id task_id;
        };
        std::vector<collision_enter_res> coll_enter;
        std::vector<collision_exit_res> coll_exit;
        std::vector<destruction_res> destructions;

        template <type T> auto &get_waiter()
        {
            if constexpr (T == type::collision_enter)
                return coll_enter;
            else if constexpr (T == type::collision_exit)
                return coll_exit;
            else
                return destructions;
        }

        [[nodiscard]] bool empty() const
        { return coll_enter.empty() && coll_exit.empty() && destructions.empty(); }
    };

    struct dependency
    {
        task_id task_id;
        handle_id_t source_object;
    };

    absl::flat_hash_map<handle_id_t, object_waiter> M_object_waiters;
    absl::flat_hash_map<handle_id_t, std::vector<dependency>> M_object_deps;

    arena<task<>> M_tasks;
    swappable_queue<task_id> M_pre_queue;
    swappable_queue<task_id> M_post_queue;
    swappable_queue<task_id> M_post_physics_queue;
    swappable_queue<task_id> M_cleanup_queue;
    swappable_queue<task_id> M_immediate_queue;

    std::vector<std::pair<mp_units::quantity<mp_units::si::second>, handle_id_t>>
        M_timer_heap; // TODO: memory management
    mp_units::quantity<mp_units::si::second> M_current_time{0.0 * mp_units::si::second};

    bool drain(swappable_queue<task_id> &queue)
    {
        return queue.drain([this](auto id) { resume_task(id); });
    }

    template <object_waiter::type T> void remove_waiter(handle_id_t object_id, handle_id_t task_id)
    {
        auto it = M_object_waiters.find(object_id);
        if (it == M_object_waiters.end()) return;
        auto &waiters = it->second.template get_waiter<T>();
        // Should be close to constant time
        std::erase_if(waiters,
                      [task_id, this](const auto &w)
                      {
                          if (w.task_id == task_id)
                          {
                              if constexpr (T == object_waiter::type::collision_enter ||
                                            T == object_waiter::type::collision_exit)
                                  if (w.with != null_id) remove_dependency(w.with, task_id);
                              return true;
                          }
                          return false;
                      });
        if (waiters.empty() && it->second.empty()) M_object_waiters.erase(it);
    }

    void remove_dependency(handle_id_t object_id, handle_id_t task_id)
    {
        if (auto it = M_object_deps.find(object_id); it != M_object_deps.end())
        {
            std::erase_if(it->second, [task_id](const auto &d) { return d.task_id == task_id; });
            if (it->second.empty()) M_object_deps.erase(it);
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

inline task_id awaiter::add_task(task<> t)
{ return handler().add_task(std::move(t), world(), {}).id(); }
inline task_id awaiter::add_task_eager(task<> t)
{ return handler().add_task_eager(std::move(t), world(), {}).id(); }
inline void awaiter::cancel_task(task_id id) { handler().cancel_task(id, {}); }

} // namespace detail
} // namespace physkit