// Coroutine-orchestrated missile demo
// Demonstrates how coroutines simplify complex multi-phase scene sequencing
// that would otherwise require brittle state machines.

#ifdef PHYSKIT_MODULES
import physkit;
import mp_units;
#else
#include <cmath>
#include <numbers>
#include <physkit/physkit.h>
#endif

#ifdef PHYSKIT_GRAPHICS_MODULES
import graphics;
#else
#include <graphics/graphics.h>
#endif

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .window_size({1280, 720})
                           .title("Coroutine Missile Demo")
                           .cam_pos(fvec3{0.0f, 18.0f, -50.0f} * si::metre)
                           .cam_dir(fvec3{0.0f, -0.3f, 1.0f} * mp_units::one)
                           .gravity(vec3{0, -9.81, 0} * si::metre / si::second / si::second)
                           .solver_iterations(10)}
    // NOLINT
    {
        using namespace mp_units::si::unit_symbols;

        // Ground
        auto ground_mesh = mesh::box(vec3{50, 0.5, 30} * m);
        auto ground = world().create_rigid(object_desc::stat()
                                               .with_pos(vec3{0, -0.5, 0} * m)
                                               .with_mesh(ground_mesh)
                                               .with_restitution(0.3)
                                               .with_friction(0.8));
        add_object(ground, {0.3f, 0.4f, 0.3f});

        // Target wall on the right side
        auto wall_mesh = mesh::box(vec3{1, 3, 4} * m);
        auto wall = world().create_rigid(object_desc::stat()
                                             .with_pos(vec3{12, 3, 0} * m)
                                             .with_mesh(wall_mesh)
                                             .with_restitution(0.2)
                                             .with_friction(0.7));
        add_object(wall, {0.55f, 0.5f, 0.45f});

        // Small bunker on the left
        auto bunker_mesh = mesh::box(vec3{2, 1.5, 3} * m);
        auto bunker = world().create_rigid(object_desc::stat()
                                               .with_pos(vec3{-10, 1.5, 0} * m)
                                               .with_mesh(bunker_mesh)
                                               .with_restitution(0.2)
                                               .with_friction(0.7));
        add_object(bunker, {0.5f, 0.45f, 0.4f});

        // Camera track to follow the action
        cam().set_move_track(camera_track({
                                              // Start: wide overview
                                              kf::make_pos(fvec3{0, 18, -60} * m)
                                                  .look_at(fvec3{0, 3, 0} * m)
                                                  .transition(2.0f * s),
                                              // Grenade throw — follow arc
                                              kf::make_pos(fvec3{-5, 20, -50} * m)
                                                  .look_at(fvec3{2, 6, 0} * m)
                                                  .transition(2.5f * s),
                                              // Grenade explosion
                                              kf::make_pos(fvec3{5, 14, -35} * m)
                                                  .look_at(fvec3{6, 10, 0} * m)
                                                  .transition(1.0f * s),
                                              // Pan to right side — missile launch
                                              kf::make_pos(fvec3{10, 14, -38} * m)
                                                  .look_at(fvec3{12, 4, 0} * m)
                                                  .transition(1.0f * s),
                                              // Follow missile flight
                                              kf::make_pos(fvec3{0, 16, -42} * m)
                                                  .look_at(fvec3{-5, 3, 0} * m)
                                                  .transition(1.0f * s),
                                              // Missile explosion — wide shot of aftermath
                                              kf::make_pos(fvec3{-2, 20, -45} * m)
                                                  .look_at(fvec3{0, 1, 0} * m)
                                                  .transition(4.0f * s),
                                              // Final overview
                                              kf::make_pos(fvec3{0, 22, -50} * m)
                                                  .look_at(fvec3{0, 0, 0} * m)
                                                  .transition(0.0f * s),
                                          })
                                 .with_extrap(camera_track::release));

        world().add_task(scene(this));
    }

    void update(mp_units::quantity<mp_units::si::second> /*dt*/) override {}

private:
    // Hide a graphics object by scaling to zero.
    // After remove_rigid(), sync() becomes a no-op so the transform persists.
    static void hide(gfx_obj *obj) { obj->resetTransformation().scale({0, 0, 0}); }

    static void spawn_debris(app *self, vec3<si::metre> origin, int count, double speed,
                             double up_speed, float debris_size, Color3 base_color)
    {
        using namespace mp_units::si::unit_symbols;
        auto dmesh = mesh::box(vec3{1, 1, 1} * debris_size * m);

        for (int i = 0; i < count; ++i)
        {
            double angle = static_cast<double>(i) * 2.0 * std::numbers::pi / count;
            double elevation = up_speed * (0.6 + 0.4 * static_cast<double>(i % 3));
            auto vel = vec3{speed * std::cos(angle), elevation, speed * std::sin(angle)} * m / s;

            auto dh = self->world().create_rigid(
                object_desc::dynam()
                    .with_pos(origin)
                    .with_vel(vel)
                    .with_ang_vel(vec3{10.0 * std::sin(angle), 5.0, 10.0 * std::cos(angle)} * rad /
                                  s)
                    .with_mass(0.05 * kg)
                    .with_mesh(dmesh)
                    .with_restitution(0.5)
                    .with_friction(0.4));

            float t = static_cast<float>(i) / static_cast<float>(count);
            Color3 color{base_color[0] + 0.15f * t, base_color[1] + 0.1f * t, base_color[2]};
            self->add_object(dh, color);
        }
    }

    static task scene(app *self)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        // --- Phase 1: Throw grenade from the left bunker ---
        (void) co_await wait_for(1.5 * s);

        auto grenade_mesh = mesh::sphere(0.3 * m, 8, 16);
        auto grenade_h = w.create_rigid(object_desc::dynam()
                                            .with_pos(vec3{-10, 3.5, 0} * m)
                                            .with_vel(vec3{11, 15, 0} * m / s)
                                            .with_ang_vel(vec3{5, 2, 3} * rad / s)
                                            .with_mass(0.5 * kg)
                                            .with_mesh(grenade_mesh)
                                            .with_restitution(0.4)
                                            .with_friction(0.5));
        auto *grenade_gfx = self->add_object(grenade_h, {0.2f, 0.35f, 0.15f});

        // --- Phase 2: Fuse timer, then detonate ---
        (void) co_await wait_for(2.0 * s);

        auto grenade_pos = (*w.get_rigid(grenade_h))->pos();
        auto _ = w.remove_rigid(grenade_h);
        hide(grenade_gfx);

        spawn_debris(self, grenade_pos, 14, 7.0, 8.0, 0.12f, {0.9f, 0.45f, 0.1f});

        // --- Phase 3: Retaliation — launch missile from the right ---
        (void) co_await wait_for(2.0 * s);

        auto missile_mesh = mesh::box(vec3{0.9, 0.2, 0.2} * m);
        auto missile_h = w.create_rigid(object_desc::dynam()
                                            .with_pos(vec3{10, 5, 0} * m)
                                            .with_vel(vec3{-20, 2, 0} * m / s)
                                            .with_mass(3.0 * kg)
                                            .with_mesh(missile_mesh)
                                            .with_restitution(0.1)
                                            .with_friction(0.3));
        auto *missile_gfx = self->add_object(missile_h, {0.65f, 0.12f, 0.08f});

        // --- Phase 4: Missile impact ---
        (void) co_await wait_for_collision(missile_h);

        auto missile_pos = (*w.get_rigid(missile_h))->pos();
        auto _ = w.remove_rigid(missile_h);
        hide(missile_gfx);

        spawn_debris(self, missile_pos, 24, 9.0, 10.0, 0.18f, {0.85f, 0.2f, 0.05f});
    }
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
