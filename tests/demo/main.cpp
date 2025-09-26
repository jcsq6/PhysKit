#include "physkit/particle.h"
#include <physkit/physkit.h>

int main()
{
    using namespace mp_units;
    using namespace si::unit_symbols;
    physkit::print_demo();
    physkit::units_demo();
    physkit::eigen_demo();

    constexpr auto g = -9.80 * m / s / s;

    physkit::particle p {
        .pos = {0.0 * m, 0.0 * m, 0.0 * m},
        .vel = {1.0 * m / s, 0.0 * m / s, 0.0 * m / s},
        .acc = {0.0 * m / s / s, 0.0 * m / s / s, g},
        .mass = 1.0 * kg
    };

    constexpr auto dt = 0.1 * s;
    std::println("Integrating with dt = {}", dt);
    physkit::forward_euler integrator;
    for (int i = 0; i < 10; ++i) {
        integrator.integrate(p, dt);
        std::println("t = {}, pos = {}, vel = {}", i * dt, p.pos, p.vel);
    }

    return 0;
}