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

using task_handle = detail::task_handler::task_handle;

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

using constraint_desc_variant =
    std::variant<impulse::distance_constraint::desc, impulse::spring_constraint::desc,
                 impulse::ball_socket_constraint::desc, impulse::hinge_constraint::desc,
                 impulse::slider_constraint::desc, impulse::weld_constraint::desc>;

template <auto Type>
using handle_for = typename physkit::detail::arena<impulse::constraint_class_type<Type>>::handle;

using constraint_handle_variant =
    std::variant<handle_for<constraint_type::distance>, handle_for<constraint_type::ball_socket>,
                 handle_for<constraint_type::hinge>, handle_for<constraint_type::slider>,
                 handle_for<constraint_type::weld>, handle_for<constraint_type::spring>>;

using constraint_variant =
    std::variant<impulse::distance_constraint, impulse::ball_socket_constraint,
                 impulse::hinge_constraint, impulse::slider_constraint, impulse::weld_constraint,
                 impulse::spring_constraint>;

struct add_constraint_command : command_base
{
    using result_type = constraint_handle_variant;
    constraint_desc_variant desc;

    add_constraint_command(result_type *storage, std::coroutine_handle<> resume_handle,
                           constraint_desc_variant desc)
        : command_base(storage, resume_handle), desc(std::move(desc))
    {
    }
};

struct remove_constraint_command : command_base
{
    using result_type = std::optional<constraint_variant>;
    constraint_handle_variant h;

    remove_constraint_command(result_type *storage, std::coroutine_handle<> resume_handle,
                              constraint_handle_variant h)
        : command_base(storage, resume_handle), h(h)
    {
    }
};

using command =
    std::variant<add_task_command, cancel_task_command, add_rigid_command, destroy_rigid_command,
                 add_constraint_command, remove_constraint_command>;
} // namespace detail

class world_base
{

public:
    explicit world_base(const world_desc &desc) : M_gravity(desc.gravity()), M_task_handler(*this)
    {
    }
    world_base(const world_base &) = delete;
    world_base &operator=(const world_base &) = delete;
    // TODO: figure out move semantics
    world_base(world_base &&) = delete;
    world_base &operator=(world_base &&) = delete;

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

    task_handle add_task(task<> t) { return M_task_handler.add_task(std::move(t), {}); }
    bool cancel_task(task_handle handle) { return M_task_handler.cancel_task(handle.id(), {}); }

    [[nodiscard]] bool task_active(task_handle handle) const
    { return M_task_handler.task_active(handle.id()); }

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

    // Raycast into world, return range of object hit and distance
    [[nodiscard]] generator<std::pair<handle, mp_units::quantity<si::metre>>>
    raycast(ray r, quantity<si::metre> max_dist) const
    {
        auto static_gen = M_broad.static_tree().raycast(r, max_dist);
        auto dynamic_gen = M_broad.dynamic_tree().raycast(r, max_dist);

        auto static_it = static_gen.begin();
        auto dynamic_it = dynamic_gen.begin();

        auto static_distance = std::numeric_limits<quantity<si::metre>>::max();
        auto dynamic_distance = std::numeric_limits<quantity<si::metre>>::max();

        auto static_handle = handle::from_id(handle::null);
        auto dynamic_handle = handle::from_id(handle::null);

        bool has_static = static_it != static_gen.end();
        bool has_dynamic = dynamic_it != dynamic_gen.end();

        auto make_value = [this](auto &&entry)
        { return std::make_pair(M_rigid.get_slot_handle(entry.first), entry.second); };

        auto fetch = [make_value](auto &gen_it, auto &handle, auto &distance)
        {
            std::tie(handle, distance) = make_value(*gen_it);
            ++gen_it;
        };

        if (has_static) fetch(static_it, static_handle, static_distance);
        if (has_dynamic) fetch(dynamic_it, dynamic_handle, dynamic_distance);

        while (has_static && has_dynamic)
        {
            if (static_distance <= dynamic_distance)
            {
                co_yield {static_handle, static_distance};
                has_static = static_it != static_gen.end();
                if (has_static) fetch(static_it, static_handle, static_distance);
            }
            else
            {
                co_yield {dynamic_handle, dynamic_distance};
                has_dynamic = dynamic_it != dynamic_gen.end();
                if (has_dynamic) fetch(dynamic_it, dynamic_handle, dynamic_distance);
            }
        }

        while (has_static)
        {
            co_yield {static_handle, static_distance};
            has_static = static_it != static_gen.end();
            if (has_static) fetch(static_it, static_handle, static_distance);
        }

        while (has_dynamic)
        {
            co_yield {dynamic_handle, dynamic_distance};
            has_dynamic = dynamic_it != dynamic_gen.end();
            if (has_dynamic) fetch(dynamic_it, dynamic_handle, dynamic_distance);
        }
    }

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
    { return M_task_handler.cancel_task(cmd.handle.id(), {}); }
    auto execute_command(detail::add_rigid_command &cmd) { return create_rigid(cmd.desc); }

    auto execute_command(detail::destroy_rigid_command &cmd) { return remove_rigid(cmd.h); }

    virtual detail::constraint_handle_variant
    execute_add_constraint(const detail::constraint_desc_variant &desc) = 0;
    virtual std::optional<detail::constraint_variant>
    execute_remove_command(const detail::constraint_handle_variant &h) = 0;

    auto execute_command(detail::add_constraint_command &cmd)
    { return execute_add_constraint(cmd.desc); }

    auto execute_command(detail::remove_constraint_command &cmd)
    { return execute_remove_command(cmd.h); }

    bool flush_commands()
    {
        return M_command_queue.drain(
            [this](auto &cmd)
            { std::visit([this](auto &&arg) { arg.store(execute_command(arg)); }, cmd); });
    }
};

using object_handle = world_base::handle;

class world : public world_base
{
public:
    explicit world(const world_desc &desc)
        : world_base(desc), M_constraints(desc.solver_iterations(), desc.gravity().norm())
    {
    }

    template <typename Desc>
    auto add_constraint(const Desc &desc)
        requires requires { Desc::build_constraint; }
    {
        auto obj_a = this->get_rigid(world_base::handle::from_id(desc.a));
        auto obj_b = this->get_rigid(world_base::handle::from_id(desc.b));
        return M_constraints.add_constraint(
            Desc::build_constraint(*obj_a.value(), *obj_b.value(), desc));
    }

    template <typename Handle> auto remove_constraint(Handle h)
    { return M_constraints.remove_constraint(h); }

private:
    impulse::constraint_solver M_constraints;

    detail::constraint_handle_variant
    execute_add_constraint(const detail::constraint_desc_variant &desc) override
    {
        return std::visit([this](auto &&d) -> detail::constraint_handle_variant
                          { return this->add_constraint(d); }, desc);
    }

    std::optional<detail::constraint_variant>
    execute_remove_command(const detail::constraint_handle_variant &h) override
    {
        return std::visit(
            [this](auto &&handle)
            {
                return this->remove_constraint(handle).transform(
                    [](auto &&c) -> detail::constraint_variant
                    { return std::forward<decltype(c)>(c); });
            },
            h);
    }

    void step_impl(quantity<si::second> dt) override;
};

} // namespace physkit