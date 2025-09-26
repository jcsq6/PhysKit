#pragma once
#include "particle.h"
#include <mp-units/framework.h>
#include <mp-units/systems/si/units.h>

namespace physkit
{
    class integrator
    {
    public:
        integrator() = default;
        integrator(const integrator&) = default;
        integrator& operator=(const integrator&) = default;
        integrator(integrator&&) = default;
        integrator& operator=(integrator&&) = default;

        virtual void integrate(particle& p, quantity<si::second> dt) = 0;
        virtual ~integrator() = default;
    };

    class forward_euler : public integrator
    {
    public:
        void integrate(particle& p, quantity<si::second> dt) override
        {
            p.vel += p.acc * dt;
            p.pos += p.vel * dt;
        }
    };
}