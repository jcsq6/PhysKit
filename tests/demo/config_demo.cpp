// Magnum header needed for MAGNUM_APPLICATION_MAIN macro (not exportable from modules)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#else
#include <physkit/physkit.h>
#endif

#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#include <graphics/graphics.h>
#endif

using namespace mp_units;
using namespace mp_units::si::unit_symbols;
using namespace physkit;
using namespace graphics;

class app : public graphics_app
{
public:
    explicit app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config{arguments}.title_or("Config Demo")}
    // NOLINT
    {
        cam().speed(10 * m / s);
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override {}
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
