#include "physkit/physkit.h"
#include <mp-units/format.h> // needed for std::println with mp-units
#include <mp-units/systems/international.h> // for miles per hour
#include <mp-units/systems/si.h> // for meters per second
#include <print>

namespace physkit
{
    void print_demo()
    {
        std::println("Hello from PhysKit, version {}!", PHYSKIT_VERSION);
    }

    void units_demo()
    {
        using namespace mp_units;
        using namespace mp_units::si::unit_symbols;
        using namespace mp_units::international::unit_symbols;

        constexpr auto c = 299'792'458.0 * m / s; 
        constexpr auto c_mph = c.force_in(mi / h);

        std::println("The speed of light is approximately {::N[.2f]}, or {::N[.2f]}.", c, c_mph);
    }

}