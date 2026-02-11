#include "physkit/world.h"

namespace physkit
{
void world::step(quantity<si::second> dt)
{
    for (auto &slot : M_rigid.slots)
    {
        if (!slot.available)
        {
            auto &obj = slot.value;
            obj.apply_force(M_gravity * obj.mass());
            M_int->integrate(obj, dt);
            obj.clear_forces();
        }
    }
}
} // namespace physkit