// GJK tests for PhysKit

#include "physkit/gjk.h"

#include <mp-units/systems/si/unit_symbols.h>

#include <cassert>
#include <cmath>
#include <print>

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

constexpr double eps = 1e-6;

bool approx(double a, double b) { return std::abs(a - b) < eps; }
bool approx_q(quantity<si::metre> a, quantity<si::metre> b)
{
    return approx(a.numerical_value_in(si::metre), b.numerical_value_in(si::metre));
}
bool approx_q(quantity<pow<2>(si::metre)> a, quantity<pow<2>(si::metre)> b)
{
    return approx(a.numerical_value_in(pow<2>(si::metre)), b.numerical_value_in(pow<2>(si::metre)));
}
bool approx_q(quantity<pow<3>(si::metre)> a, quantity<pow<3>(si::metre)> b)
{
    return approx(a.numerical_value_in(pow<3>(si::metre)), b.numerical_value_in(pow<3>(si::metre)));
}

bool approx_vec(const vec3<si::metre> &a, const vec3<si::metre> &b)
{
    return approx_q(a.x(), b.x()) && approx_q(a.y(), b.y()) && approx_q(a.z(), b.z());
}

// ---------------------------------------------------------------------------
// GJK tests
// ---------------------------------------------------------------------------

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main() {}