#pragma once
#include "detail/types.h"
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
struct particle
{
    vec3<si::metre> pos;
    vec3<si::metre / si::second> vel;
    vec3<si::metre / si::second / si::second> acc;
    quantity<si::kilogram> mass = 1.0 * si::kilogram;
};
} // namespace physkit