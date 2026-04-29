// Magnum header needed for MAGNUM_APPLICATION_MAIN macro (not exportable from modules)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <array>
#include <cmath>
#include <coroutine> // IWYU pragma: keep
#include <random>
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

class fountain_app : public graphics_app
{
    static inline const auto gravity_a = vec3{0.0, -9.81, 0.0} * m / s / s;
    static constexpr auto ball_radius = 0.18 * m;
    static constexpr auto ball_mass = 0.05 * kg;
    static constexpr auto despawn_y = -8.0 * m;
    static constexpr auto spawn_period = 0.0035 * s;
    static constexpr std::size_t max_balls = 600;

public:
    explicit fountain_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .title("PhysKit Fountain Waterfall")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{50.0f, 35.0f, 0.0f} * si::metre)
                           .look_at(fvec3{0.0f, 4.0f, -1.0f} * si::metre)
                           .drag(false)
                           .gravity(gravity_a)
                           .time_step(1.0 / 360.0 * si::second)
                           .solver_iterations(6)}
    { world().add_task(scene()); }

    void update(quantity<si::second> /*dt*/) override {}

private:
    task<> scene()
    {
        const Color3 stone_dark{0.40f, 0.36f, 0.32f};
        const Color3 stone_med{0.55f, 0.50f, 0.45f};
        const Color3 step_color{0.45f, 0.55f, 0.62f};
        const Color3 basin_color{0.30f, 0.32f, 0.38f};

        // --- Mountain pillar (fountain base) ---
        co_await add_rigid(object_desc::stat()
                               .with_pos(vec3{0, 4.5, -12} * m)
                               .with_shape(box(vec3{2.0, 4.5, 1.5} * m))
                               .with_friction(0.6),
                           stone_dark);

        // Spout cap on top of pillar
        co_await add_rigid(object_desc::stat()
                               .with_pos(vec3{0, 9.3, -12} * m)
                               .with_shape(box(vec3{1.2, 0.3, 1.2} * m))
                               .with_friction(0.4),
                           stone_med);

        // Containment box around the spout, open on the +z/front side.
        constexpr double spout_box_half_x = 1.55;
        constexpr double spout_box_half_y = 1.05;
        constexpr double spout_box_half_z = 1.25;
        constexpr double spout_box_thick = 0.18;
        const vec3 spout_box_center = vec3{0.0, 10.05, -12.0} * m;

        co_await add_rigid(object_desc::stat()
                               .with_pos(spout_box_center +
                                         vec3{0.0, (spout_box_half_y + spout_box_thick), 0.0} * m)
                               .with_shape(box(vec3{spout_box_half_x + spout_box_thick,
                                                    spout_box_thick, spout_box_half_z} *
                                               m))
                               .with_friction(0.5),
                           stone_med);
        // Back wall
        co_await add_rigid(
            object_desc::stat()
                .with_pos(spout_box_center +
                          vec3{0.0, 0.0, -(spout_box_half_z + spout_box_thick)} * m)
                .with_shape(box(vec3{spout_box_half_x, spout_box_half_y, spout_box_thick} * m))
                .with_friction(0.5),
            stone_med);
        // Side walls
        for (double sign : {-1.0, 1.0})
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(spout_box_center +
                              vec3{sign * (spout_box_half_x + spout_box_thick), 0.0, 0.0} * m)
                    .with_shape(box(vec3{spout_box_thick, spout_box_half_y, spout_box_half_z} * m))
                    .with_friction(0.5),
                stone_med);

        // --- Waterfall steps: tilted slabs cascading toward the basin ---
        struct step_def
        {
            vec3<si::metre> pos;
            vec3<si::metre> half;
            quantity<si::degree> tilt;
        };
        const std::array<step_def, 3> steps{{
            {.pos = vec3{0, 7.5, -7.0} * m, .half = vec3{3.5, 0.25, 2.5} * m, .tilt = 10 * deg},
            {.pos = vec3{0, 4.5, -2.5} * m, .half = vec3{3.5, 0.25, 2.5} * m, .tilt = 14 * deg},
            {.pos = vec3{0, 1.5, 2.0} * m, .half = vec3{3.5, 0.25, 2.5} * m, .tilt = 18 * deg},
        }};
        for (const auto &s : steps)
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(s.pos)
                    .with_orientation(quat<one>::from_angle_axis(s.tilt, vec3<one>{1.0, 0.0, 0.0}))
                    .with_shape(box(s.half))
                    .with_restitution(0.05)
                    .with_friction(0.15),
                step_color);

        // --- Bottom basin: square pool with drain hole at center ---
        constexpr double basin_cz = 8.0;
        constexpr double basin_half = 5.0;
        constexpr double drain_half = 1.0;
        constexpr double floor_y = -0.5;
        constexpr double floor_thick = 0.3;
        constexpr double wall_height = 1.4;
        constexpr double wall_thick = 0.3;

        constexpr double strip_long_half = (basin_half - drain_half) / 2.0;

        // Long floor strips on +/-z sides of drain (full x span)
        for (double sign : {-1.0, 1.0})
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(
                        vec3{0.0, floor_y, basin_cz + (sign * (drain_half + strip_long_half))} * m)
                    .with_shape(box(vec3{basin_half, floor_thick, strip_long_half} * m))
                    .with_friction(0.4),
                basin_color);
        // Short floor strips on +/-x sides between drain and walls
        for (double sign : {-1.0, 1.0})
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(vec3{sign * (drain_half + strip_long_half), floor_y, basin_cz} * m)
                    .with_shape(box(vec3{strip_long_half, floor_thick, drain_half} * m))
                    .with_friction(0.4),
                basin_color);

        // Walls — tall on the +z side of basin (to catch overshoot), shorter rim elsewhere.
        const double y_wall = floor_y + floor_thick + wall_height;
        // North/south long walls (x-running)
        for (double sign : {-1.0, 1.0})
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(vec3{0.0, y_wall, basin_cz + (sign * (basin_half + wall_thick))} * m)
                    .with_shape(
                        box(vec3{basin_half + (2.0 * wall_thick), wall_height, wall_thick} * m))
                    .with_friction(0.5),
                stone_med);
        // East/west walls (z-running)
        for (double sign : {-1.0, 1.0})
            co_await add_rigid(
                object_desc::stat()
                    .with_pos(vec3{sign * (basin_half + wall_thick), y_wall, basin_cz} * m)
                    .with_shape(box(vec3{wall_thick, wall_height, basin_half} * m))
                    .with_friction(0.5),
                stone_med);

        co_await spawn_loop();
    }

    static task<> track_ball(physics_obj *ball)
    {
        co_await wait_until([ball] { return ball->obj().pos().y() < despawn_y; });
        delete ball;
    }

    task<> spawn_loop()
    {
        std::mt19937 rng{0x5EEDu};
        std::uniform_real_distribution<double> jitter{-1.0, 1.0};

        const auto sphere_shape = sphere(ball_radius);
        const auto spout_pos = vec3{0.0, 10.5, -12.0} * m;

        while (true)
        {
            const auto vx = jitter(rng) * 0.5;
            const auto vy = 5.5 + (std::abs(jitter(rng)) * 0.8);
            const auto vz = 2.0 + (jitter(rng) * 0.3);
            const auto pos = spout_pos + vec3{jitter(rng) * 0.15, 0.0, jitter(rng) * 0.15} * m;

            const float t = 0.5f * (1.0f + static_cast<float>(jitter(rng)));
            const Color3 color{0.20f + (0.10f * t), 0.55f + (0.20f * t), 0.85f + (0.10f * t)};

            co_await add_task<policy::no_wait>(
                track_ball(*co_await add_rigid(object_desc::dynam()
                                                   .with_pos(pos)
                                                   .with_vel(vec3{vx, vy, vz} * m / s)
                                                   .with_mass(ball_mass)
                                                   .with_shape(sphere_shape)
                                                   .with_restitution(0.10)
                                                   .with_friction(0.20),
                                               color)));
            co_await wait_for(spawn_period);
        }
    }
};

MAGNUM_APPLICATION_MAIN(fountain_app) // NOLINT
