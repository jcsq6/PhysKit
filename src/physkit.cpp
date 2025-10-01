#include "physkit/physkit.h"

#include <print>

namespace physkit
{
void print_demo() { std::println("Hello from PhysKit, version {}!", PHYSKIT_VERSION); }

void units_demo()
{
    using namespace si::unit_symbols;
    using namespace international::unit_symbols;

    constexpr auto c = 299'792'458.0 * m / s;
    constexpr auto c_mph = c.force_in(mi / h);

    std::println("The speed of light is approximately {::N[.2f]}, or {::N[.2f]}.", c, c_mph);
}

void eigen_demo()
{
    Eigen::Vector3d x(1, 0, 0);
    Eigen::Vector3d y(0, 1, 0);

    auto z = x.cross(y);

    std::println("x = {}, y = {}, z = x \u2A2F y = {}", x, y, z);
    Eigen::Matrix3d r;
    r << x, y, z;
    std::println("Matrix [x, y, z]:\n{}", r);
}
} // namespace physkit