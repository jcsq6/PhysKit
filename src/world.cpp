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
            obj.particle().apply_force(M_gravity * obj.particle().mass());
            M_int->integrate(obj, dt);
            obj.particle().clear_forces();
        }
    }
}
} // namespace physkit