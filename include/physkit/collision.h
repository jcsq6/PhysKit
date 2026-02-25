#pragma once

#include "detail/types.h"
#include "mesh.h"

#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
class collision_info // TODO: time of impact for continuous collision detection
{
public:
    vec3<one> normal = vec3<one>::zero();
    vec3<si::metre> local_a = vec3<si::metre>::zero();
    vec3<si::metre> local_b = vec3<si::metre>::zero();
    quantity<si::metre> depth{};
};

std::optional<collision_info> gjk_epa(const mesh::instance &a, const mesh::instance &b);
std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b);
} // namespace physkit