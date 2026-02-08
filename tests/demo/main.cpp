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

    auto obj = physkit::object(physkit::object_desc::dynam()
                                   .with_pos({0.0 * m, 0.0 * m, 0.0 * m})
                                   .with_vel({1.0 * m / s, 0.0 * m / s, 0.0 * m / s})
                                   .with_mass(1.0 * kg));
    obj.particle().acc = {0.0 * m / s / s, 0.0 * m / s / s, g};

    constexpr auto dt = 0.1 * s;
    std::println("Integrating with dt = {}", dt);
    physkit::forward_euler integrator;
    for (int i = 0; i < 10; ++i) // NOLINT
    {
        integrator.integrate(obj, dt);
        auto &p = obj.particle();
        std::println("t = {::N[.2f]}, pos = {:.2f}, vel = {:.2f}", i * dt, p.pos, p.vel);
    }

    auto &p = obj.particle();
    std::formatter<quantity<si::metre>> fmt;

    // demonstrate unit_mat
    std::println("\nunit_mat demo:");
    std::println("pos.x = {::N[.2f]}", p.pos.x());
    std::println("cross(pos, vel) = {:.2f}", p.pos.cross(p.vel));
    std::println("dot(pos, vel) = {::N[.2f]}", p.pos.dot(p.vel));
    std::println("norm(pos) = {::N[.2f]}", p.pos.norm());
    physkit::mat3<si::metre> M = physkit::mat3<si::metre>::identity(); // NOLINT
    std::println("M = \n{:.2f}", M);
    std::println("M^T = \n{:.2f}", M.transpose());
    std::println("det(M) = {::N[.2f]}", M.determinant());
    std::println("trace(M) = {::N[.2f]}", M.trace());
    std::println("M^-1 = \n{:.2f}", M.inverse());
    std::println("norm(M) = {::N[.2f]}", M.norm());
    std::println("squared_norm(M) = {::N[.2f]}", M.squared_norm());
    std::println("M.normalized() = \n{:.2f}", M.normalized());
    std::println("M * 2.0 = \n{:.2f}", M * 2);
    std::println("2.0 * M = \n{:.2f}", 2 * M);
    std::println("M / 2.0 = \n{:.2f}", M / 2);
    std::println("M * 2.0 m = \n{:.2f}", M * (2.0 * m)); // NOLINT
    std::println("M / 2.0 m = \n{:.2f}", M / (2.0 * m)); // NOLINT
    std::println("M + M = \n{:.2f}", M + M);
    std::println("M - M = \n{:.2f}", M - M);
    std::println("M[1, 1] * 5 = {::N[.2f]}", M[1, 1] * 5); // NOLINT
    M.set(1, 1, 10 * m);                                   // NOLINT
    std::println("M[1, 1] => 10 = {::N[.2f]}", M[1, 1]);

    const auto const_M = M; // NOLINT
    std::println("const M[1, 1] = {::N[.2f]}", const_M[1, 1]);
    std::println("const M[1, 1] * 5 = {::N[.2f]}", const_M[1, 1] * 5); // NOLINT
    // std::println("const M[1, 1] => 10 = {::N[.2f]}", const_M[1, 1] = 10 * m); // should not
    // compile
    std::println("const M[1, 1] < 5 * M[1, 1] = {}", const_M[1, 1] < const_M[1, 1]);

    return 0;
}