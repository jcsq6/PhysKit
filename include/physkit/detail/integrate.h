#pragma once

#ifdef PHYSKIT_IN_MODULE_IMPL
#ifdef PHYSKIT_IMPORT_STD
import std;
#endif
#else
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>
#endif

#include "../core/object.h"

PHYSKIT_EXPORT
namespace physkit
{

namespace detail
{
inline auto exp(const vec3<si::radian / si::second> &ang_vel, quantity<si::second> dt)
{
    auto angle = ang_vel * dt;
    auto angle_mag = angle.norm();
    if (angle_mag < 1e-12 * si::radian)
    {
        auto half = angle * 0.5;
        return quat{1.0 * si::radian, half.x(), half.y(), half.z()}
            .normalized(); // Small angle approx breaks type system
    }
    return quat<one>::from_angle_axis(angle_mag, angle / angle_mag);
}
} // namespace detail

struct semi_implicit_euler
{
    static void integrate_vel(object &obj, quantity<si::second> dt)
    {
        obj.vel() += obj.acc() * dt;
        obj.ang_vel() += obj.angular_accel() * dt;
    }

    static void integrate_pos(object &obj, quantity<si::second> dt)
    {
        obj.pos() += obj.vel() * dt;
        obj.orientation(detail::exp(obj.ang_vel(), dt) * obj.orientation());
    }
};

} // namespace physkit
