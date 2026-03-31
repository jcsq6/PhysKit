#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/core/world.h"
#endif

namespace physkit
{
namespace detail
{
task_handler &awaiter::handler() const
{
    assert(M_promise.world != nullptr);
    return M_promise.world->handler({});
}

has_waiter_field *task_handler::get_waiter_fields(handle_id_t object_id)
{ return M_world->get_waiter_fields(object_id, {}); }
} // namespace detail
template class world<semi_implicit_euler>;

template <> void world<semi_implicit_euler>::step_impl(const quantity<si::second> dt)
{
    for (auto &slot : rigids().slots)
    {
        if (slot.available() || slot.value->obj.is_static()) continue;

        auto &obj = slot.value->obj;
        obj.apply_force(gravity() * obj.mass());
        semi_implicit_euler::integrate_vel(obj, dt);

        broad_phase().update_node(slot.value->broad_handle, obj.instance().bounds(),
                                  obj.vel() * dt);

        obj.clear_forces();
    }

    broad_phase().calculate_pairs(
        narrow_phase(),
        [this](auto h) -> std::pair<detail::dynamic_bvh::node_handle, bool>
        {
            auto &slot = rigids().get_slot(h);
            return {slot.value->broad_handle, slot.value->obj.is_static()};
        });

    narrow_phase().calculate([this](auto h) -> object & { return rigids().get_slot(h).value->obj; },
                             [this](auto &man_info) { on_collision(man_info); },
                             [this](auto a, auto b) { on_collision_exit(a, b); });
    M_constraints.setup_contacts(dt, narrow_phase(), [this](auto h) -> object &
                                 { return rigids().get_slot(h).value->obj; });
    M_constraints.solve_constraints(dt);
    for (auto &slot : rigids().slots)
    {
        if (slot.available() || slot.value->obj.is_static()) continue;

        semi_implicit_euler::integrate_pos(slot.value->obj, dt);
    }
}
} // namespace physkit