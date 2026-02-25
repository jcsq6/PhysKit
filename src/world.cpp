#include "physkit/world.h"
// TODO: remove
#include "physkit/detail/eigen_format.h" // IWYU pragma: keep
#include <print>

namespace physkit
{
void world::step(quantity<si::second> dt)
{
    for (auto &slot : M_rigid.slots)
    {
        if (slot.available || slot.obj.is_static()) continue;

        auto &obj = slot.obj;
        obj.apply_force(M_gravity * obj.mass());
        M_int->integrate(obj, dt);

        M_broad.update_node(slot.broad_handle, obj.instance().bounds(), obj.vel() * dt);

        obj.clear_forces();
    }

    M_broad.calculate_pairs(M_narrow,
                            [this](auto h) -> std::pair<detail::dynamic_bvh::node_handle, bool>
                            {
                                auto &slot = M_rigid.get_slot(h);
                                return {slot.broad_handle, slot.obj.is_static()};
                            });

    M_narrow.calculate([this](auto h) -> object & { return M_rigid.get_slot(h).obj; });
    for (const auto &man : M_narrow.manifolds())
    {
        if (!man.man) continue;
        auto &obj_a = M_rigid.get_slot(man.a).obj;
        auto &obj_b = M_rigid.get_slot(man.b).obj;

        std::println("Collision detected between contacts {}\n",
                     man.man.contacts() | std::views::transform(
                                              [](const auto &c)
                                              {
                                                  return std::tuple(c.info.local_a, c.info.local_b,
                                                                    c.info.normal, c.info.depth);
                                              }));
    }
}
} // namespace physkit