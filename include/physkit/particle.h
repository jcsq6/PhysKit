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

    vec3<si::radian / si::second> ang_vel;
    vec3<si::radian / si::second / si::second> ang_acc;

    void apply_force(const vec3<si::kilogram * si::metre / si::second / si::second> &force)
    {
        acc += force / mass;
    }

    void clear_forces()
    {
        acc = vec3<si::metre / si::second / si::second>::zero();
        ang_acc = vec3<si::radian / si::second / si::second>::zero();
    }
};
} // namespace physkit