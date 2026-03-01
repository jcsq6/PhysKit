#include <mp-units/systems/si/unit_symbols.h>
#include <physkit/physkit.h>

#include <graphics.h>

using namespace mp_units;
using namespace si::unit_symbols;
using namespace physkit;
using namespace graphics;

class app : public graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics_app{g_config{arguments}.title_or("Config Demo")}
    // NOLINT
    { cam().speed(10 * m / s); }

    void update(physkit::quantity<physkit::si::second> dt) override {}
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
