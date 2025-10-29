#include "graphics.h"
#include <mp-units/systems/si/units.h>

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics::graphics_app{arguments, {800, 600}, 45 * si::degree, "PhysKit Graphics Test"}
    {
        
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
    }
private:
    physics_obj *M_cube{};
    gfx_obj *M_sphere{};
};

MAGNUM_APPLICATION_MAIN(app)