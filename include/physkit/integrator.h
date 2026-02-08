#pragma once
#include "object.h"
#include <cassert>
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{

namespace detail
{
/// @brief Build a skew-symmetric matrix from an angular displacement vector.
/// Treats radian values as dimensionless (physically correct for small-angle rotation).
inline mat3<one> skew(const vec3<si::radian / si::second> &ang_vel, quantity<si::second> dt)
{
    auto wx = (ang_vel.x() * dt).numerical_value_in(si::radian);
    auto wy = (ang_vel.y() * dt).numerical_value_in(si::radian);
    auto wz = (ang_vel.z() * dt).numerical_value_in(si::radian);

    auto sk = mat3<one>::zero();
    sk.set(0, 1, -wz * one);
    sk.set(0, 2, wy * one);
    sk.set(1, 0, wz * one);
    sk.set(1, 2, -wx * one);
    sk.set(2, 0, -wy * one);
    sk.set(2, 1, wx * one);
    return sk;
}
} // namespace detail

class integrator
{
public:
    integrator() = default;
    integrator(const integrator &) = default;
    integrator &operator=(const integrator &) = default;
    integrator(integrator &&) = default;
    integrator &operator=(integrator &&) = default;

    virtual void integrate(object &obj, quantity<si::second> dt) = 0;
    virtual ~integrator() = default;
};

class forward_euler : public integrator
{
public:
    void integrate(object &obj, quantity<si::second> dt) override
    {
        auto &p = obj.particle();
        p.pos += p.vel * dt;
        p.vel += p.acc * dt;

        obj.orientation() += detail::skew(p.ang_vel, dt) * obj.orientation();
        p.ang_vel += p.ang_acc * dt;
    }
};

class semi_implicit_euler : public integrator
{
public:
    void integrate(object &obj, quantity<si::second> dt) override
    {
        auto &p = obj.particle();
        p.vel += p.acc * dt;
        p.pos += p.vel * dt;

        p.ang_vel += p.ang_acc * dt;
        obj.orientation() += detail::skew(p.ang_vel, dt) * obj.orientation();
    }
};

class velocity_verlet : public integrator
{
public:
    void integrate(object & /*obj*/, quantity<si::second> /*dt*/) override
    {
        // TODO: implement velocity Verlet integration
        assert(false && "velocity_verlet::integrate not yet implemented");
    }
};

class rk4 : public integrator
{
public:
    void integrate(object & /*obj*/, quantity<si::second> /*dt*/) override
    {
        // TODO: implement RK4 integration
        assert(false && "rk4::integrate not yet implemented");
    }
};

} // namespace physkit
