#pragma once
#include "world.h"

namespace physkit
{
class fixed_accum
{
public:
    fixed_accum(quantity<si::second> fixed_dt) : M_dt(fixed_dt) {}

    void update(quantity<si::second> dt) { M_accum += dt; }

    [[nodiscard]] bool should_step()
    {
        if (M_accum >= M_dt)
        {
            M_accum -= M_dt;
            return true;
        }
        return false;
    }

    [[nodiscard]] auto step_size() const { return M_dt; }
    [[nodiscard]] auto accum() const { return M_accum; }
    [[nodiscard]] auto alpha() const { return M_accum / M_dt; }

private:
    quantity<si::second> M_dt;
    quantity<si::second> M_accum = 0 * si::second;
};

class stepper
{
public:
    stepper(world &w, quantity<si::second> dt) : M_accum(dt), M_w(&w) {}

    void update(quantity<si::second> dt)
    {
        M_accum.update(dt);
        while (M_accum.should_step()) M_w->step(M_accum.step_size());
    }

    [[nodiscard]] auto accum() const { return M_accum; }
    [[nodiscard]] auto step_size() const { return M_accum.step_size(); }
    [[nodiscard]] auto alpha() const { return M_accum.alpha(); }

private:
    fixed_accum M_accum;
    world *M_w;
};

} // namespace physkit