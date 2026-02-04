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

        auto phys_obj2 = std::make_unique<physkit::object>(
            physkit::particle{
                .pos = {0.0 * m, 0.0 * m, 0.0 * m},
                .vel = {0.0 * m / s, 0.0 * m / s, 0.0 * m / s},
                .acc = {0.0 * m / s / s, 0.0 * m / s / s, 0.0 * m / s / s},
                .mass = 1.0 * kg,
            },
            std::make_shared<physkit::mesh>());

        M_cube = add_object(std::move(phys_obj), std::move(cube_mesh), {0.0, 1.0, 0.0});
        M_cube2 = add_object(std::move(phys_obj2), mesh_objs::cube(), {1.0, 0.0, 0.0});
        M_sphere->translate({3.0f, 0.0f, 0.0f});
        //M_cube2->translate({0.0f, 3.0f, 0.0f});

        cam().speed(8.0f * m / s);
        track t {
            .points = {
                {0.0f * m, 0.0f * m, 0.0f * m}, {-10.0f * m, -10.0f * m, -10.0f * m},
                {0.0f * m, -10.0f * m, -10.0f * m}
            },
            .times = {0.0f * s, 6.0f * s, 7.0f * s},
            .interpolation = track::spline,
        };


        cam().set_move_track(t);
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        using namespace si::unit_symbols;

        constexpr auto rotation_speed = std::numbers::pi_v<float> * rad / s;

        M_sphere->rotate(rotation_speed * dt, {0.0f, 1.0f, 0.0f});
        M_cube->rotate(rotation_speed * dt, {1.0f, 1.0f, 0.0f});
        
        cam().look_at(vec3(0.0f * m, 0.0f * m, 0.0f * m));
    }

private:
    physics_obj *M_cube{};
    physics_obj *M_cube2{};
    gfx_obj *M_sphere{};
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT