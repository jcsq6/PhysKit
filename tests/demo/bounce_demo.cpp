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
        : graphics_app{g_config{arguments, false}
                           .cam_pos_or(vec3{0, 0, -25} * m)
                           .cam_dir_or(vec3{0, 0, 1} * one)
                           .title_or("PhysKit Demo")
                           .window_size_or({1280, 720})}
    {
        auto cube_mesh = mesh::box(vec3{1, 1, 1} * m);
        M_a = add_object(world().create_rigid(object_desc::dynam()
                                                  .with_pos(vec3{0, 15, 0} * m)
                                                  .with_vel(vec3{0, 0, 0} * m / s)
                                                  .with_mass(1 * kg)
                                                  .with_mesh(cube_mesh)),
                         {0.8f, 0.2f, 0.2f});

        auto *ground = add_object(mesh_objs::cube(), {0.2f, 0.35f, 0.2f});
        ground->scale({200.0f, 0.05f, 200.0f});
        ground->translate({0.0f, -0.05f, 0.0f}); // top face at y = 0

        cam().speed(10 * m / s);
        cam().look_at({0 * m, M_a->obj().pos().y(), 0 * m});
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        auto &obj = M_a->obj();
        if (obj.pos().y() < 0 * m)
        {
            obj.pos().y(-obj.pos().y());
            obj.vel().y(-obj.vel().y());
        }
    }

private:
    physics_obj *M_a;
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
