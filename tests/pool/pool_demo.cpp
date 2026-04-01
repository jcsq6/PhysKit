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

class pool_app : public graphics_app
{
public:
    explicit pool_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .title("PhysKit Pool Demo")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{8.0f, 7.0f, -4.0f} * si::metre)
                           .look_at(fvec3{0.0f, 0.0f, 1.5f} * si::metre)
                           .drag(true)
                           .gravity(vec3{0.0, -9.81, 0.0} * m / s / s)
                           .solver_iterations(15)}
    {
        build_table();
        build_balls();
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override
    {
        if (M_charging) M_charge_time += dt;
    }

    void pointer_press(PointerEvent &event, bool pressed) override
    {
        if (event.pointer() != Pointer::MouseLeft) return;
        if (pressed)
        {
            M_charging = true;
            M_charge_time = 0.0 * s;
        }
        else if (M_charging)
        {
            M_charging = false;
            shoot();
        }
    }

private:
    physics_obj *M_cue{};
    bool M_charging{false};
    quantity<si::second> M_charge_time{0.0 * s};

    void build_table()
    {
        // Felt surface
        auto felt = object_desc::stat()
                        .with_mesh(mesh::box(vec3{3.0, 0.1, 5.0} * m))
                        .with_pos(vec3{0.0, -0.1, 0.5} * m)
                        .with_restitution(0.3)
                        .with_friction(0.6);
        add_object(world().create_rigid(felt), Color3{0.1f, 0.45f, 0.15f});

        // Rails
        auto rail_fb = mesh::box(vec3{3.1, 0.2, 0.1} * m);
        auto rail_lr = mesh::box(vec3{0.1, 0.2, 5.1} * m);
        Color3 wood{0.45f, 0.25f, 0.1f};

        auto make_rail = [&](auto m, vec3<si::metre> pos)
        {
            add_object(world().create_rigid(object_desc::stat()
                                                .with_mesh(m)
                                                .with_pos(pos)
                                                .with_restitution(0.75)
                                                .with_friction(0.3)),
                       wood);
        };
        make_rail(rail_fb, vec3{0.0, 0.1, -4.6} * m);
        make_rail(rail_fb, vec3{0.0, 0.1, 5.6} * m);
        make_rail(rail_lr, vec3{-3.1, 0.1, 0.5} * m);
        make_rail(rail_lr, vec3{3.1, 0.1, 0.5} * m);
    }

    void build_balls()
    {
        auto ball = mesh::sphere(0.3 * m, 16, 24);
        auto mass = 0.17 * kg;

        auto make_ball = [&](vec3<si::metre> pos, Color3 color)
        {
            return add_object(world().create_rigid(object_desc::dynam()
                                                       .with_mesh(ball)
                                                       .with_pos(pos)
                                                       .with_mass(mass)
                                                       .with_restitution(0.95)
                                                       .with_friction(0.05)),
                              color);
        };

        // Cue ball
        M_cue = make_ball(vec3{0.0, 0.3, -2.5} * m, {0.95f, 0.95f, 0.95f});

        // Rack: 5 rows, apex at z=2.0
        constexpr double sp = 0.601;       // just over diameter
        constexpr double zsp = sp * 0.866; // equilateral triangle row spacing
        constexpr double z0 = 2.0;
        constexpr double y = 0.3;

        // Row 1
        make_ball(vec3{0.0, y, z0} * m, {1.0f, 0.85f, 0.0f});
        // Row 2
        make_ball(vec3{-sp / 2, y, z0 + zsp} * m, {1.0f, 0.92f, 0.55f});
        make_ball(vec3{sp / 2, y, z0 + zsp} * m, {0.0f, 0.25f, 0.85f});
        // Row 3
        make_ball(vec3{-sp, y, z0 + zsp * 2} * m, {0.55f, 0.65f, 0.9f});
        make_ball(vec3{0.0, y, z0 + zsp * 2} * m, {0.08f, 0.08f, 0.08f}); // 8-ball
        make_ball(vec3{sp, y, z0 + zsp * 2} * m, {0.85f, 0.1f, 0.1f});
        // Row 4
        make_ball(vec3{-sp * 1.5, y, z0 + zsp * 3} * m, {0.9f, 0.55f, 0.55f});
        make_ball(vec3{-sp / 2, y, z0 + zsp * 3} * m, {0.5f, 0.1f, 0.1f});
        make_ball(vec3{sp / 2, y, z0 + zsp * 3} * m, {0.5f, 0.0f, 0.5f});
        make_ball(vec3{sp * 1.5, y, z0 + zsp * 3} * m, {0.7f, 0.45f, 0.7f});
        // Row 5
        make_ball(vec3{-sp * 2, y, z0 + zsp * 4} * m, {0.95f, 0.5f, 0.0f});
        make_ball(vec3{-sp, y, z0 + zsp * 4} * m, {0.95f, 0.72f, 0.45f});
        make_ball(vec3{0.0, y, z0 + zsp * 4} * m, {0.0f, 0.5f, 0.1f});
        make_ball(vec3{sp, y, z0 + zsp * 4} * m, {0.55f, 0.75f, 0.55f});
        make_ball(vec3{sp * 2, y, z0 + zsp * 4} * m, {0.7f, 0.45f, 0.45f});
    }

    void shoot()
    {
        if (!M_cue) return;
        auto &obj = M_cue->obj();

        // Get camera forward direction projected onto XZ plane
        Matrix4 inv = cam().inverse_view_matrix();
        Vector3 fwd = inv.transformVector(-Vector3::zAxis());
        fwd.y() = 0.0f;
        if (fwd.dot() < 1e-6f) return;
        fwd = fwd.normalized();

        double t = std::min(M_charge_time.numerical_value_in(s), 2.0);
        auto speed = (1.0 + t * 5.0) * m / s;
        auto dir = vec3{double(fwd.x()), 0.0, double(fwd.z())};
        obj.apply_impulse(dir * (obj.mass() * speed));
    }
};

MAGNUM_APPLICATION_MAIN(pool_app) // NOLINT
