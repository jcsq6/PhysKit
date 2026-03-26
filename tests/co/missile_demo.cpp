// Coroutine-orchestrated missile demo
// Demonstrates how sub-task coroutines compose complex multi-phase scene sequencing:
//   - task<T> sub-tasks that return values (grenade detonation position flows into missile
//   targeting)
//   - Nested sub-task composition (launch_guided → guide_to_target)
//   - Per-frame steering via next_frame inside a sub-task
//   - Fire-and-forget parallel task spawning (MIRV sub-munitions via add_task)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>

#ifndef PHYSKIT_IMPORT_STD
#include <cmath>
#include <coroutine> // IWYU pragma: keep
#include <numbers>
#include <print>
#endif
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

        // Dynamic camera track — keyframes are appended by the scene coroutine
        cam().set_move_track(
            camera_track(kf::make_pos(fvec3{0, 18, -60} * m).look_at(fvec3{0, 3, 0} * m))
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
            double elevation = up_speed * (0.6 + (0.4 * static_cast<double>(i % 3)));
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
            Color3 color{base_color[0] + (0.15f * t), base_color[1] + (0.1f * t), base_color[2]};
            self->add_object(dh, color);
        }
    }

    // -----------------------------------------------------------------------
    // Sub-task coroutines — each encapsulates one logical phase as a reusable
    // building block that can be co_await-ed from the main scene.
    // -----------------------------------------------------------------------

    /// Throw a grenade with a timed fuse.  Returns the detonation position.
    static task<vec3<si::metre>> throw_grenade(app *self, vec3<si::metre> origin,
                                               vec3<si::metre / si::second> velocity,
                                               mp_units::quantity<si::second> fuse)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        auto gmesh = mesh::sphere(0.3 * m, 8, 16);
        auto gh = w.create_rigid(object_desc::dynam()
                                     .with_pos(origin)
                                     .with_vel(velocity)
                                     .with_ang_vel(vec3{5, 2, 3} * rad / s)
                                     .with_mass(0.5 * kg)
                                     .with_mesh(gmesh)
                                     .with_restitution(0.4)
                                     .with_friction(0.5));
        auto *gfx = self->add_object(gh, {0.2f, 0.35f, 0.15f});

        co_await wait_for(fuse);

        auto pos = (*w.get_rigid(gh))->pos();
        auto _ = w.remove_rigid(gh);
        hide(gfx);
        spawn_debris(self, pos, 14, 7.0, 8.0, 0.12f, {0.9f, 0.45f, 0.1f});
        co_return pos;
    }

    /// Steer a rigid body toward `target` each frame.
    static task<vec3<si::metre>> guide_to_target(app *self, world_base::handle handle, gfx_obj *gfx,
                                                 vec3<si::metre> target, int debris_count,
                                                 float debris_size, Color3 debris_color)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        auto time_s = 0.0 * s;
        constexpr auto corrective_accel_factor = 5 / s / s;
        constexpr auto desired_accel_delta = 20 * m / s / s;
        auto desired_speed = (*w.get_rigid(handle))->vel().norm();

        auto end = [&](auto &obj)
        {
            auto _ = w.remove_rigid(handle);
            hide(gfx);
            spawn_debris(self, obj.pos(), debris_count, 8.0, 9.0, debris_size, debris_color);
            return obj.pos();
        };
        while (true)
        {
            auto res = co_await after_any{next_frame{}, wait_for_collision{.object = handle}};
            auto rigid = w.get_rigid(handle);
            auto *obj = *rigid;
            if (!rigid) co_return target;
            if (res.index() == 1) co_return end(*obj);

            auto dt = std::get<0>(res);
            time_s += dt;

            desired_speed += desired_accel_delta * dt;
            auto to_target = target - obj->pos();
            auto dist = to_target.norm();

            if (dist < 2.0 * m) co_return end(*obj);

            auto desired_vel = (to_target / dist) * desired_speed;
            auto vel_error = desired_vel - obj->vel();
            auto correction_factor = time_s * corrective_accel_factor;

            obj->apply_acceleration(vel_error * correction_factor);
        }
    }

    /// Launch a guided cruise missile from `origin` toward `target`.
    /// Composes object creation + guide_to_target sub-task.
    /// Returns the impact position.
    static task<vec3<si::metre>>
    launch_guided(app *self, vec3<si::metre> origin, vec3<si::metre> target,
                  vec3<si::metre / si::second> initial_vel = vec3{0, 0, 0} * si::metre / si::second,
                  bool explode_on_impact = false)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        auto mmesh = mesh::box(vec3{0.6, 0.15, 0.15} * m);

        auto mh = w.create_rigid(object_desc::dynam()
                                     .with_pos(origin)
                                     .with_vel(initial_vel)
                                     .with_mass(2.0 * kg)
                                     .with_mesh(mmesh)
                                     .with_restitution(0.1)
                                     .with_friction(0.3));
        auto *gfx = self->add_object(mh, {0.8f, 0.3f, 0.1f});

        // Delegate steering to a nested sub-task
        co_return co_await guide_to_target(self, mh, gfx, target, 20, 0.15f, {0.95f, 0.5f, 0.1f});
    }

    /// Fire-and-forget guided sub-munition (spawned via add_task).
    static task<> submunition(app *self, vec3<si::metre> origin, vec3<si::metre> target,
                              vec3<si::metre / si::second> initial_vel, Color3 body_color)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        auto mmesh = mesh::box(vec3{0.4, 0.1, 0.1} * m);

        auto mh = w.create_rigid(object_desc::dynam()
                                     .with_pos(origin)
                                     .with_vel(initial_vel)
                                     .with_mass(1.0 * kg)
                                     .with_mesh(mmesh)
                                     .with_restitution(0.1)
                                     .with_friction(0.3));
        auto *gfx = self->add_object(mh, body_color);

        // Reuse guide_to_target — even fire-and-forget tasks compose sub-tasks
        co_await guide_to_target(self, mh, gfx, target, 12, 0.12f, {0.85f, 0.2f, 0.05f});
    }

    /// MIRV strike: launch carrier upward, split at apex into guided sub-munitions.
    /// Sub-munitions are spawned as independent fire-and-forget tasks via add_task.
    static task<> mirv_strike(app *self, vec3<si::metre> launch_pos, vec3<si::metre> target_a,
                              vec3<si::metre> target_b, vec3<si::metre> target_c)
    {
        using namespace mp_units::si::unit_symbols;
        auto &w = self->world();

        // Launch carrier rocket upward
        auto carrier_mesh = mesh::box(vec3{0.25, .75, 0.25} * m);
        auto carrier_h = w.create_rigid(object_desc::dynam()
                                            .with_pos(launch_pos)
                                            .with_vel(vec3{0, 20, 0} * m / s)
                                            .with_mass(5.0 * kg)
                                            .with_mesh(carrier_mesh)
                                            .with_restitution(0.1)
                                            .with_friction(0.3));
        auto *carrier_gfx = self->add_object(carrier_h, {0.4f, 0.4f, 0.5f});

        // Wait for apex — vertical velocity flips sign
        while (true)
        {
            co_await next_frame{};
            auto rigid = w.get_rigid(carrier_h);
            if (!rigid) co_return;
            if ((*rigid)->vel().y() <= 0 * m / s) break;
        }

        // Remove carrier at apex
        auto apex_pos = (*w.get_rigid(carrier_h))->pos();
        auto _ = w.remove_rigid(carrier_h);
        hide(carrier_gfx);

        // Flash at split point
        spawn_debris(self, apex_pos, 6, 3.0, 2.0, 0.06f, {1.0f, 0.9f, 0.3f});

        co_await after_all{submunition(self, apex_pos + vec3{0, 0, -.5} * m, target_a,
                                       vec3{0, -2, -18} * m / s, {0.9f, 0.15f, 0.05f}),
                           submunition(self, apex_pos + vec3{0, 0, .5} * m, target_b,
                                       vec3{0, -2, 18} * m / s, {0.85f, 0.25f, 0.1f}),
                           submunition(self, apex_pos + vec3{-.5, 0, 0} * m, target_c,
                                       vec3{-18, -2, 0} * m / s, {0.95f, 0.1f, 0.0f})};
    }

    // -----------------------------------------------------------------------
    // Main scene — reads like a screenplay, each phase is a single co_await.
    // -----------------------------------------------------------------------
    static task<> scene(app *self)
    {
        using namespace mp_units::si::unit_symbols;
        auto &track = self->cam().move_track();

        // --- Phase 1: Mortar barrage from the bunker toward the target wall ---
        //     Each grenade is a sub-task returning its detonation position.
        track.append(
            kf::make_pos(fvec3{-5, 20, -50} * m).look_at(fvec3{2, 8, 0} * m).transition(1.5f * s));
        co_await wait_for(1.5 * s);

        auto crater1 =
            co_await throw_grenade(self, vec3{-10, 3.5, 0} * m, vec3{12, 10, 2} * m / s, 2.2 * s);
        std::println("Grenade 1 crater at: {}", crater1);

        // Pan toward missile area while grenade 2 flies
        track.append(
            kf::make_pos(fvec3{12, 12, -40} * m).look_at(fvec3{5, 4, 0} * m).transition(3.5f * s));

        auto crater2 = co_await throw_grenade(self, vec3{-10, 2.5, 0} * m,
                                              vec3{14, 10.4, -1} * m / s, 2.1 * s);
        std::println("Grenade 2 crater at: {}", crater2);

        // --- Phase 2: Guided cruise missile retaliates ---
        //     Targets the bunker that fired the grenades.
        //     launch_guided internally co_awaits guide_to_target (nested sub-task).
        track.append(
            kf::make_pos(fvec3{0, 16, -42} * m).look_at(fvec3{0, 3, 0} * m).transition(2.0f * s));
        co_await wait_for(1.0 * s);

        auto bunker_pos = vec3{-10.0, 1.5, 0.0} * m;
        auto impact = co_await launch_guided(self, vec3{14, 5, 0} * m, bunker_pos,
                                             vec3{0, 20, 0} * m / s, true);
        std::println("Missile impact at: {}", impact);

        // --- Phase 3: MIRV strike finishing off the bunker area ---
        //     Carrier splits into 3 independently guided sub-munitions
        //     spawned as fire-and-forget tasks via world().add_task().
        track.append(
            kf::make_pos(fvec3{0, 20, -55} * m).look_at(fvec3{0, 15, 0} * m).transition(2.0f * s));
        track.append(
            kf::make_pos(fvec3{0, 30, -50} * m).look_at(fvec3{0, 10, 0} * m).transition(3.0f * s));
        track.append(
            kf::make_pos(fvec3{0, 22, -50} * m).look_at(fvec3{0, 3, 0} * m).transition(2.0f * s));
        track.append(
            kf::make_pos(fvec3{0, 25, -55} * m).look_at(fvec3{0, 0, 0} * m).transition(3.0f * s));
        co_await wait_for(1.5 * s);

        co_await mirv_strike(self, vec3{12, 1, -12} * m, vec3{-12, 1.5, 2} * m,
                             vec3{-8, 1.5, -2} * m, vec3{-10, 1.5, 0} * m);
    }
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
