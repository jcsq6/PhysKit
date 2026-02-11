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
    static constexpr auto dt = 1.0 / 60.0f * s;
    explicit app(const Arguments &arguments)
        : graphics_app{arguments,
                       {1280, 720},
                       45 * deg,
                       "PhysKit Demo",
                       fvec3{0.0f, 10.0f, -25.0f} * m,
                       fvec3{0.0f, 0.0f, 0.0f},
                       false}
    // NOLINT
    {
        auto cube_mesh = mesh::box(vec3{1, 1, 1} * m);
        M_a = add_object(M_world,
                         M_world.create_rigid(object_desc::dynam()
                                                  .with_pos(vec3{0, 15, 0} * m)
                                                  .with_vel(vec3{0, 0, 0} * m / s)
                                                  .with_mass(1 * kg)
                                                  .with_mesh(cube_mesh)),
                         {0.8f, 0.2f, 0.2f});

        auto *ground = add_object(mesh_objs::cube(), {0.2f, 0.35f, 0.2f});
        ground->scale({200.0f, 0.05f, 200.0f});
        ground->translate({0.0f, -0.05f, 0.0f}); // top face at y = 0

        set_vsync();
        cam().speed(10 * m / s);
        cam().look_at({0 * m, M_a->obj().particle().pos().y(), 0 * m});
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        M_stepper.update(dt);
        auto &particle = M_a->obj().particle();
        if (particle.pos().y() < 0 * m)
        {
            particle.pos().y(-particle.pos().y());
            particle.vel().y(-particle.vel().y());
        }
    }

private:
    world M_world{world_desc::make()
                      .with_gravity(vec3{0, -9.8, 0} * m / s / s)
                      .with_integrator(world_desc::semi_implicit_euler)};
    stepper M_stepper{M_world, dt};
    physics_obj *M_a;
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
