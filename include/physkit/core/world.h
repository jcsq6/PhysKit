#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include <concepts>
#include <cstddef>
#include <utility>
#endif

#include "../collision/collision_phases.h"
#include "../collision/constraint.h"
#include "../detail/arena.h"
#include "co.h"
#include "integrator.h"
#include "object.h"

PHYSKIT_EXPORT
namespace physkit
{

class world_desc
{
public:
    static world_desc make() { return {}; }

    [[nodiscard]] auto gravity() const { return M_gravity; }
    auto &&with_gravity(this auto &&self, const vec3<si::metre / si::second / si::second> &gravity)
    {
        self.M_gravity = gravity;
        return std::forward<decltype(self)>(self);
    }

    [[nodiscard]] auto solver_iterations() const { return M_solver_iterations; }
    auto &&with_solver_iterations(this auto &&self, std::size_t iterations)
    {
        self.M_solver_iterations = iterations;
        return std::forward<decltype(self)>(self);
    }

private:
    vec3<si::metre / si::second / si::second> M_gravity = {
        0.0 * si::metre / si::second / si::second,
        -9.81 * si::metre / si::second / si::second,
        0.0 * si::metre / si::second / si::second,
    };
    std::size_t M_solver_iterations = 10;
};

namespace detail
{
struct obj_node
{
    object obj;
    detail::dynamic_bvh::node_handle broad_handle{};
    has_waiter_field event_waiters;
};

using object_handle = arena<obj_node>::handle;

struct command_base
{
    command_base(void *storage, std::coroutine_handle<> resume_handle)
        : storage(storage), resume_handle(resume_handle)
    {
    }

    template <typename Self>
    void store(this Self &self, typename Self::result_type &&result)
        requires(!std::same_as<typename Self::result_type, void>)
    {
        if (self.storage)
            *static_cast<typename Self::result_type *>(self.storage) = std::move(result);
        if (self.resume_handle) self.resume_handle.resume();
    }

    template <typename Self>
    void store(this Self &self, bool success)
        requires(std::same_as<typename Self::result_type, void>)
    {
        if (self.storage) *static_cast<bool *>(self.storage) = success;
        if (self.resume_handle) self.resume_handle.resume();
    }

    void *storage{};
    std::coroutine_handle<> resume_handle;
};
struct add_task_command : command_base
{
    using result_type = task_handler::task_handle;
    task<> t;

    add_task_command(result_type *storage, std::coroutine_handle<> resume_handle, task<> t)
        : command_base(storage, resume_handle), t(std::move(t))
    {
    }
};

struct cancel_task_command : command_base
{
    using result_type = bool;
    task_handler::task_handle handle;

    cancel_task_command(void *storage, std::coroutine_handle<> resume_handle,
                        task_handler::task_handle handle)
        : command_base(storage, resume_handle), handle(handle)
    {
    }
};

struct add_rigid_command : command_base
{
    using result_type = object_handle;
    object_desc desc;

    add_rigid_command(result_type *storage, std::coroutine_handle<> resume_handle, object_desc desc)
        : command_base(storage, resume_handle), desc(std::move(desc))
    {
    }
};

struct destroy_rigid_command : command_base
{
    using result_type = std::optional<object>;
    object_handle h;

    destroy_rigid_command(result_type *storage, std::coroutine_handle<> resume_handle,
                          object_handle h)
        : command_base(storage, resume_handle), h(h)
    {
    }
};

using command =
    std::variant<add_task_command, cancel_task_command, add_rigid_command, destroy_rigid_command>;
} // namespace detail

class world_base
{

public:
    explicit world_base(const world_desc &desc) : M_gravity(desc.gravity()), M_task_handler(*this)
    {
    }
    world_base(const world_base &) = delete;
    world_base &operator=(const world_base &) = delete;
    world_base(world_base &&) = default;
    world_base &operator=(world_base &&) = default;

    using handle = detail::object_handle;

    void push_command(detail::command &&cmd) { M_command_queue.push(std::move(cmd)); }

    // TODO: hide?
    auto create_rigid(const object_desc &desc)
    {
        object obj(desc);
        auto bh =
            M_broad.add({M_rigid.next_handle().index()}, obj.instance().bounds(), obj.is_static());
        return M_rigid.add({.obj = std::move(obj), .broad_handle = bh});
    }
    std::optional<object> remove_rigid(handle h)
    {
        if (auto res = M_rigid.remove(h))
        {
            M_task_handler.on_destruction(h.id(), {});
            M_broad.remove(res->broad_handle, res->obj.is_static(), M_narrow);
            return std::move(res->obj);
        }
        return std::nullopt;
    }

    auto get_rigid(this auto &&self, handle h) -> std::optional<decltype(&self.M_rigid.get(h)->obj)>
    {
        if (auto *obj = self.M_rigid.get(h)) return &obj->obj;
        return std::nullopt;
    }

    [[nodiscard]] quantity<si::second> time() const { return M_task_handler.time(); }

    void add_task(task<> t) { M_task_handler.add_task(std::move(t), {}); }

    void step(quantity<si::second> dt)
    {
        M_task_handler.increment(dt, {});
        M_task_handler.dispatch_pre({});
        flush_commands();
        step_impl(dt);
        M_task_handler.dispatch_post({});
        M_task_handler.dispatch_cleanup([this] { return flush_commands(); }, {});
    }

    detail::task_handler &handler(detail::passkey<detail::awaiter> /*key*/)
    { return M_task_handler; }

    // No check for object validity
    detail::has_waiter_field *get_waiter_fields(detail::handle_id_t object_id,
                                                detail::passkey<detail::task_handler> /*key*/)
    {
        auto *res = M_rigid.get(detail::object_handle::from_id(object_id));
        return (res != nullptr) ? &res->event_waiters : nullptr;
    }

    virtual ~world_base() = default;

    [[nodiscard]] auto &gravity() const { return M_gravity; }

protected:
    auto &rigids(this auto &&self) { return self.M_rigid; }
    auto &broad_phase(this auto &&self) { return self.M_broad; }
    auto &narrow_phase(this auto &&self) { return self.M_narrow; }
    auto &gravity(this auto &&self) { return self.M_gravity; }

    void on_collision(const detail::narrow_phase::manifold_info &man_info)
    {
        M_task_handler.on_collision(man_info, M_rigid.get_slot_handle(man_info.a).id(),
                                    M_rigid.get_slot_handle(man_info.b).id(), {});
    }

    void on_collision_exit(detail::dynamic_bvh::object_handle a,
                           detail::dynamic_bvh::object_handle b)
    {
        M_task_handler.on_collision_exit(M_rigid.get_slot_handle(a).id(),
                                         M_rigid.get_slot_handle(b).id(), {});
    }

    virtual void step_impl(quantity<si::second> dt) = 0;

private:
    detail::arena<detail::obj_node> M_rigid;
    detail::task_handler M_task_handler;
    detail::broad_phase M_broad;
    detail::narrow_phase M_narrow;
    detail::swappable_queue<detail::command> M_command_queue;
    vec3<si::metre / si::second / si::second> M_gravity;

    auto execute_command(detail::add_task_command &cmd)
    { return M_task_handler.add_task(std::move(cmd.t), {}); }
    bool execute_command(detail::cancel_task_command &cmd)
    {
        M_task_handler.cancel_task(cmd.handle.id(), {});
        return true;
    }
    auto execute_command(detail::add_rigid_command &cmd) { return create_rigid(cmd.desc); }

    auto execute_command(detail::destroy_rigid_command &cmd) { return remove_rigid(cmd.h); }

    bool flush_commands()
    {
        return M_command_queue.drain(
            [this](auto &cmd)
            { std::visit([this](auto &&arg) { arg.store(execute_command(arg)); }, cmd); });
    }
};

using object_handle = world_base::handle;

template <std::derived_from<integrator> Integrator> class world : public world_base
{
public:
    explicit world(const world_desc &desc)
        : world_base(desc), M_constraints(desc.solver_iterations(), desc.gravity().norm())
    {
    }

    template <typename T> auto add_constraint(const T &con)
    { return M_constraints.template add_constraint<T::static_type>(con); }

private:
    impulse::constraint_solver<Integrator> M_constraints;

    void step_impl(quantity<si::second> dt) override;
};

} // namespace physkit