#include "graphics.h"
#include "physkit/particle.h"
#include <memory>
#include <mp-units/systems/si/units.h>

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics::graphics_app{arguments,
                                 {800, 600},
                                 45 * si::degree,
                                 "PhysKit Graphics Test",
                                 {0.0f, 5.0f, -10.0f},
                                 {0.0f, -5.0f, 10.0f}}
    // NOLINT
    {
        auto cube_mesh = mesh_objs::cube();
        auto sphere_mesh = mesh_objs::sphere();

        using namespace si::unit_symbols;

        M_sphere = add_object(std::move(sphere_mesh), {1.0, 0.0, 0.0});
        auto phys_obj = std::make_unique<physkit::object>(
            physkit::particle{
                .pos = {0.0 * m, 0.0 * m, 0.0 * m},
                .vel = {0.0 * m / s, 0.0 * m / s, 0.0 * m / s},
                .acc = {0.0 * m / s / s, 0.0 * m / s / s, 0.0 * m / s / s},
                .mass = 1.0 * kg,
            },
            std::make_shared<physkit::mesh>());
        M_cube = add_object(std::move(phys_obj), std::move(cube_mesh), {0.0, 1.0, 0.0});
        M_sphere->translate({2.0f, 5.0f, 0.0f});

        cam().speed(2.5f * si::metre / si::second);
    }

    void update(physkit::quantity<physkit::si::second> dt) override {}

private:
    physics_obj *M_cube{};
    gfx_obj *M_sphere{};
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT