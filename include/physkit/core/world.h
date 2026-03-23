#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include <concepts>
#include <cstddef>
#include <cstdint>
#include <expected>
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

class world_base
{
    struct obj_node
    {
        object obj;
        detail::dynamic_bvh::node_handle broad_handle{};
    };

    using task_id = detail::arena<task>::handle::id_type;

public:
    explicit world_base(const world_desc &desc) : M_gravity(desc.gravity()) {}
    world_base(const world_base &) = delete;
    world_base &operator=(const world_base &) = delete;
    world_base(world_base &&) = default;
    world_base &operator=(world_base &&) = default;

    using handle = detail::arena<obj_node>::handle;
    enum class err_t : std::uint8_t
    {
        stale_handle
    };

    auto create_rigid(const object_desc &desc)
    {
        object obj(desc);
        auto bh =
            M_broad.add({M_rigid.next_handle().index()}, obj.instance().bounds(), obj.is_static());
        return M_rigid.add({.obj = std::move(obj), .broad_handle = bh});
    }
    std::expected<object, err_t> remove_rigid(handle h)
    {
        if (auto res = M_rigid.remove(h))
        {
            M_broad.remove(res->broad_handle, res->obj.is_static(), M_narrow);
            return std::move(res->obj);
        }
        return std::unexpected(err_t::stale_handle);
    }
    auto get_rigid(this auto &&self, handle h)
        -> std::expected<decltype(&self.M_rigid.get(h)->obj), err_t>
    {
        if (auto *obj = self.M_rigid.get(h)) return &obj->obj;
        return std::unexpected(err_t::stale_handle);
    }

    [[nodiscard]] quantity<si::second> time() const { return M_task_handler.time(); }

    void add_task(task t) { M_task_handler.add_task(std::move(t), *this, {}); }

    void step(quantity<si::second> dt)
    {
        M_task_handler.process_tasks({});
        step_impl(dt);
        M_task_handler.increment(dt, {});
    }

    detail::task_handler &handler(detail::passkey<detail::awaiter> /*key*/)
    { return M_task_handler; }

    virtual ~world_base() = default;

protected:
    auto &rigids(this auto &&self) { return self.M_rigid; }
    auto &broad_phase(this auto &&self) { return self.M_broad; }
    auto &narrow_phase(this auto &&self) { return self.M_narrow; }
    auto &gravity(this auto &&self) { return self.M_gravity; }

    void on_collision(const detail::narrow_phase::manifold_info &man_info)
    {
        M_task_handler.on_collision(man_info, M_rigid.get_slot_handle(man_info.a).id(),
                                    M_rigid.get_slot_handle(man_info.b).id());
    }

    void on_collision_exit(detail::dynamic_bvh::object_handle a,
                           detail::dynamic_bvh::object_handle b)
    {
        M_task_handler.on_collision_exit(M_rigid.get_slot_handle(a).id(),
                                         M_rigid.get_slot_handle(b).id());
    }

    virtual void step_impl(quantity<si::second> dt) = 0;

private:
    detail::task_handler M_task_handler;
    detail::arena<obj_node> M_rigid;
    detail::broad_phase M_broad;
    detail::narrow_phase M_narrow;
    vec3<si::metre / si::second / si::second> M_gravity;
};

using object_handle = world_base::handle;

template <std::derived_from<integrator> Integrator> class world : public world_base
{
public:
    explicit world(const world_desc &desc)
        : world_base(desc), M_constraints(desc.solver_iterations())
    {
    }

    template <typename T> auto add_constraint(const T &con)
    { return M_constraints.template add_constraint<T::static_type>(con); }

private:
    impulse::constraint_solver<Integrator> M_constraints;

    void step_impl(quantity<si::second> dt) override;
};

} // namespace physkit