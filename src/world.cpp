#ifndef PHYSKIT_IN_MODULE_IMPL
#include "physkit/core/world.h"
#endif

namespace physkit
{
namespace detail
{

[[nodiscard]] task_scheduler &awaiter::scheduler() const
{ return world().scheduler(task::_passkey({})); }

void awaiter::queue_task(world_base::task_id id) const
{ world().queue_task(id, task::_passkey({})); }
} // namespace detail

template class world<semi_implicit_euler>;

template <> void world<semi_implicit_euler>::step_impl(quantity<si::second> dt)
{
    for (auto &slot : rigids().slots)
    {
        if (slot.available || slot.value.obj.is_static()) continue;

        auto &obj = slot.value.obj;
        obj.apply_force(gravity() * obj.mass());
        semi_implicit_euler::integrate_vel(obj, dt);

        broad_phase().update_node(slot.value.broad_handle, obj.instance().bounds(), obj.vel() * dt);

        obj.clear_forces();
    }

    broad_phase().calculate_pairs(
        narrow_phase(),
        [this](auto h) -> std::pair<detail::dynamic_bvh::node_handle, bool>
        {
            auto &slot = rigids().get_slot(h);
            return {slot.value.broad_handle, slot.value.obj.is_static()};
        });

    narrow_phase().calculate([this](auto h) -> object & { return rigids().get_slot(h).value.obj; });
    M_constraints.setup_contacts(dt, narrow_phase(), [this](auto h) -> object &
                                 { return rigids().get_slot(h).value.obj; });
    M_constraints.solve_constraints(dt);
    for (auto &slot : rigids().slots)
    {
        if (slot.available || slot.value.obj.is_static()) continue;

        semi_implicit_euler::integrate_pos(slot.value.obj, dt);
    }
}
} // namespace physkit