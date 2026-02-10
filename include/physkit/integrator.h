#pragma once
#include "object.h"
#include <cassert>
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{

namespace detail
{
inline auto exp(const uvec3<si::radian / si::second> &ang_vel, quantity<si::second> dt)
{
    auto angle = ang_vel * dt;
    auto angle_mag = angle.norm();
    if (angle_mag < 1e-6 * si::radian)
    {
        auto half = angle * 0.5;
        return uquat{1.0 * si::radian, half.x(), half.y(), half.z()}
            .normalized(); // Small angle approx breaks type system
    }
    return uquat<one>::from_angle_axis(angle_mag, angle / angle_mag);
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

        p.pos() += p.vel() * dt;
        p.vel() += p.acc() * dt;

        auto ang_acc = p.angular_accel();

        p.orientation() = detail::exp(p.ang_vel(), dt) * p.orientation();
        p.ang_vel() += ang_acc * dt;
    }
};

class semi_implicit_euler : public integrator
{
public:
    void integrate(object &obj, quantity<si::second> dt) override
    {
        auto &p = obj.particle();

        p.vel() += p.acc() * dt;
        p.pos() += p.vel() * dt;

        p.ang_vel() += p.angular_accel() * dt;
        p.orientation() = detail::exp(p.ang_vel(), dt) * p.orientation();
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
