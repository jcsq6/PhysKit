#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL

#ifdef PHYSKIT_IMPORT_STD
import std;
#endif

#else
#include <mp-units/framework.h>
#include <mp-units/math.h>
#include <mp-units/systems/si/units.h>
#include <optional>
#endif

#include "../algebra/types.h"
#include "shape.h"

PHYSKIT_EXPORT
namespace physkit
{
class collision_info
{
public:
    vec3<one> normal = vec3<one>::zero();
    vec3<si::metre> world_a = vec3<si::metre>::zero();
    vec3<si::metre> world_b = vec3<si::metre>::zero();
    quantity<si::metre> depth{};
};

std::optional<collision_info> gjk_epa(const physkit::instance &a, const physkit::instance &b);
// std::optional<collision_info> sat(const mesh::instance &a, const mesh::instance &b);
} // namespace physkit
