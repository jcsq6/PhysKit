#include "physkit/particle.h"
#include <mp-units/systems/si/unit_symbols.h>
#include <physkit/physkit.h>
#include <print>

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
    for (int i = 0; i < 10; ++i) { // NOLINT
        integrator.integrate(p, dt);
        std::println("t = {}, pos = {}, vel = {}", i * dt, p.pos, p.vel);
    }

    // demonstrate unit_mat
    std::println("\nunit_mat demo:");
    std::println("pos.x = {}", p.pos.x());
    std::println("cross(pos, vel) = {}", p.pos.cross(p.vel));
    std::println("dot(pos, vel) = {}", p.pos.dot(p.vel));
    std::println("norm(pos) = {}", p.pos.norm());
    physkit::mat3<si::metre> M = physkit::mat3<si::metre>::identity(); // NOLINT
    std::println("M = \n{}", M);
    std::println("M^T = \n{}", M.transpose());
    std::println("det(M) = {}", M.determinant());
    std::println("trace(M) = {}", M.trace());
    std::println("M^-1 = \n{}", M.inverse());
    std::println("norm(M) = {}", M.norm());
    std::println("squared_norm(M) = {}", M.squared_norm());
    std::println("M.normalized() = \n{}", M.normalized());
    std::println("M * 2.0 = \n{}", M * 2);
    std::println("M / 2.0 = \n{}", M / 2);
    std::println("M * 2.0 m = \n{}", M * (2.0 * m)); // NOLINT
    std::println("M / 2.0 m = \n{}", M / (2.0 * m)); // NOLINT
    std::println("M + M = \n{}", M + M);
    std::println("M - M = \n{}", M - M);
    std::println("M[1, 1] * 5 = {}", M[1, 1] * 5); // NOLINT
    std::println("M[1, 1] => 10 = {}", M[1, 1] = 10 * m); // NOLINT

    const auto const_M = M; // NOLINT
    std::println("const M[1, 1] = {}", const_M[1, 1]);
    std::println("const M[1, 1] * 5 = {}", const_M[1, 1] * 5); // NOLINT
    // std::println("const M[1, 1] => 10 = {}", const_M[1, 1] = 10 * m); // should not compile
    std::println("const M[1, 1] < 5 * M[1, 1] = {}", const_M[1, 1] < const_M[1, 1]);

    return 0;
}