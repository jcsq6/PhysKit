// Magnum header needed for MAGNUM_APPLICATION_MAIN macro (not exportable from modules)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
#include <optional>
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
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world;

public:
    explicit pool_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .title("PhysKit Pool Demo")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{0.0f, 2.0f, -3.0f} * si::metre)
                           .look_at(fvec3{0.0f, 0.0f, 1.5f} * si::metre)
                           .drag(false)
                           .gravity(gravity)
                           .time_step(1.0 / 1200.0 * si::second)
                           .solver_iterations(20)}
    {
        cam().speed(1.0f * si::metre / si::second);
        world().add_task(scene());
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override {}

private:
    static task<> respawn_cue_ball(world_base::handle cue_ball_handle)
    {
        while (true)
        {
            co_await wait_for(2.0 * si::second);
            auto cue_ball_opt = co_await get_rigid(cue_ball_handle);
            if (cue_ball_opt)
            {
                auto &cue_ball = **cue_ball_opt;
                if (cue_ball.pos().y() < -2.0 * m)
                {
                    cue_ball.pos() = vec3{0.0 * m, 0.5 * m, -0.8 * m};
                    cue_ball.vel() = vec3<si::metre / si::second>::zero();
                    cue_ball.ang_vel() = vec3<one / si::second>::zero();
                }
            }
        }
    }

    // Teleport the invisible anchor to the desired camera-relative position.
    // The stick follows it kinematically during aiming.
    void update_anchor(quantity<mp_units::si::second> dt, quantity<si::metre> shoot_offset,
                       object &stick, object &anchor)
    {
        auto bounds = stick.shape().bounds();
        auto stick_half_len = (bounds.max.y() - bounds.min.y()) * 0.5f;

        auto fwd = cam().forward();
        auto tip_z = 1.0f * m; // Keep resting position anchor stationary
        auto stick_half_len_float = value_cast<float>(stick_half_len);
        auto target_pos = cam().pos() +
                          fwd * (tip_z + value_cast<float>(shoot_offset) - stick_half_len_float) +
                          cam().up() * -0.05f * m;

        auto magnum_fwd = to_magnum_vector<one, float>(fwd);
        auto target_rot =
            to_physkit_quaternion<one, double>(Quaternion::rotation(Vector3::yAxis(), -magnum_fwd));

        anchor.pos() = target_pos;
        anchor.orientation(target_rot);
    }

    task<> make_rail(physkit::shape msh, vec3<si::metre> pos, Color3 color)
    {
        co_await add_rigid(object_desc::stat()
                               .with_shape(std::move(msh))
                               .with_pos(pos)
                               .with_restitution(0.75)
                               .with_friction(0.3),
                           color);
    }

    task<world_base::handle> make_ball(physkit::shape ball,
                                       physkit::quantity<si::kilogram, double> ball_mass,
                                       vec3<si::metre> pos, Color3 color)
    {
        co_return (*co_await add_rigid(object_desc::dynam()
                                           .with_shape(std::move(ball))
                                           .with_pos(pos)
                                           .with_mass(ball_mass)
                                           .with_restitution(0.9)
                                           .with_friction(0.005),
                                       color))
            ->handle();
    }

    task<> scene()
    {
        // --- Anchor (invisible static body that drives the stick via weld constraint) ---
        auto initial_pos = vec3{0.0, 5.0, 0.0} * m;
        auto anchor_handle =
            *co_await create_rigid(object_desc::stat()
                                       .with_shape(box(vec3{0.001, 0.001, 0.001} * m))
                                       .with_pos(initial_pos));

        // --- Stick ---
        auto stick_handle = (*co_await add_rigid(object_desc::dynam()
                                                     .with_shape(box(vec3{0.005, 0.75, 0.005} * m))
                                                     .with_pos(initial_pos)
                                                     .with_mass(.5 * kg)
                                                     .with_restitution(1)
                                                     .with_friction(0.05),
                                                 Color3{0.72f, 0.53f, 0.2f}))
                                ->handle();

        // --- Constraint ---
        auto &anchor_obj = **co_await get_rigid(anchor_handle);
        auto &stick_obj = **co_await get_rigid(stick_handle);

        // Slider keeps the stick structurally aligned with the camera vector (Z-axis local)
        co_await physkit::add_constraint{
            impulse::slider_constraint::desc::make(anchor_handle, stick_handle)
                .with_anchor(initial_pos)
                .with_axis(vec3<one>{0.0, 1.0, 0.0})};

        // Soft spring keeps it tethered elastically, pulling the stick through dynamic physics
        co_await physkit::add_constraint{
            impulse::spring_constraint::desc::make(anchor_handle, stick_handle)
                .with_local_anchor_a(anchor_obj.project_to_local(initial_pos))
                .with_local_anchor_b(stick_obj.project_to_local(initial_pos))
                .with_distance(0.0 * m)
                .with_stiffness(200.0 * kg / s / s)
                .with_damping(15.0 * kg / s)};

        // --- Table ---
        auto felt_mesh = box(vec3{0.8, 0.1, 1.6} * m);
        auto felt_pos = vec3{0.0, -0.1, 0.0} * m;

        co_await add_rigid(object_desc::stat()
                               .with_shape(felt_mesh)
                               .with_pos(felt_pos)
                               .with_restitution(0.3)
                               .with_friction(0.6),
                           Color3{0.1f, 0.45f, 0.15f});

        auto felt_bounds = felt_mesh.bounds();
        auto felt_hx = (felt_bounds.max.x() - felt_bounds.min.x()) * 0.5;
        auto felt_hz = (felt_bounds.max.z() - felt_bounds.min.z()) * 0.5;

        auto rail_t = 0.05 * m;   // rail half-thickness
        auto rail_h = 0.05 * m;   // rail half-height
        auto pocket_r = 0.07 * m; // size of the pocket gaps

        // Top of felt is at y = 0.0m
        // We center rails at y = 0.0m so their tops are at 0.05m
        auto rail_fb_hx = felt_hx - pocket_r;
        auto rail_fb = box(vec3{rail_fb_hx, rail_h, rail_t});

        auto rail_lr_hz = (felt_hz - 2.0 * pocket_r) * 0.5;
        auto rail_lr = box(vec3{rail_t, rail_h, rail_lr_hz});
        auto lr_z_offset = pocket_r + rail_lr_hz;

        Color3 wood{0.45f, 0.25f, 0.1f};

        co_await make_rail(rail_fb, vec3{0.0 * m, 0.0 * m, -felt_hz - rail_t}, wood);
        co_await make_rail(rail_fb, vec3{0.0 * m, 0.0 * m, felt_hz + rail_t}, wood);

        co_await make_rail(rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, -lr_z_offset}, wood);
        co_await make_rail(rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, lr_z_offset}, wood);
        co_await make_rail(rail_lr, vec3{felt_hx + rail_t, 0.0 * m, -lr_z_offset}, wood);
        co_await make_rail(rail_lr, vec3{felt_hx + rail_t, 0.0 * m, lr_z_offset}, wood);

        // --- Balls ---
        auto ball = sphere(0.0285 * m);
        auto ball_mass = 0.17 * kg;

        // Cue ball
        auto ball_bounds = ball.bounds();
        auto ball_radius = (ball_bounds.max.x() - ball_bounds.min.x()) * 0.5;
        double sp = (ball_bounds.max.x() - ball_bounds.min.x()).numerical_value_in(m) +
                    0.001; // just over diameter

        auto cue_ball_handle = *co_await make_ball(
            ball, ball_mass, vec3{0.0 * m, ball_radius, -0.8 * m}, {0.95f, 0.95f, 0.95f});

        // Rack: 5 rows, apex at z=0.8
        double zsp = sp * 0.866; // equilateral triangle row spacing
        double z0 = 0.8;
        auto y = ball_radius;

        // Row 1
        co_await make_ball(ball, ball_mass, vec3{0.0 * m, y, z0 * m}, {1.0f, 0.85f, 0.0f});
        // Row 2
        co_await make_ball(ball, ball_mass, vec3{-sp / 2 * m, y, (z0 + zsp) * m},
                           {1.0f, 0.92f, 0.55f});
        co_await make_ball(ball, ball_mass, vec3{sp / 2 * m, y, (z0 + zsp) * m},
                           {0.0f, 0.25f, 0.85f});
        // Row 3
        co_await make_ball(ball, ball_mass, vec3{-sp * m, y, (z0 + zsp * 2) * m},
                           {0.55f, 0.65f, 0.9f});
        co_await make_ball(ball, ball_mass, vec3{0.0 * m, y, (z0 + zsp * 2) * m},
                           {0.08f, 0.08f, 0.08f}); // 8-ball
        co_await make_ball(ball, ball_mass, vec3{sp * m, y, (z0 + zsp * 2) * m},
                           {0.85f, 0.1f, 0.1f});
        // Row 4
        co_await make_ball(ball, ball_mass, vec3{-sp * 1.5 * m, y, (z0 + zsp * 3) * m},
                           {0.9f, 0.55f, 0.55f});
        co_await make_ball(ball, ball_mass, vec3{-sp / 2 * m, y, (z0 + zsp * 3) * m},
                           {0.5f, 0.1f, 0.1f});
        co_await make_ball(ball, ball_mass, vec3{sp / 2 * m, y, (z0 + zsp * 3) * m},
                           {0.5f, 0.0f, 0.5f});
        co_await make_ball(ball, ball_mass, vec3{sp * 1.5 * m, y, (z0 + zsp * 3) * m},
                           {0.7f, 0.45f, 0.7f});
        // Row 5
        co_await make_ball(ball, ball_mass, vec3{-sp * 2 * m, y, (z0 + zsp * 4) * m},
                           {0.95f, 0.5f, 0.0f});
        co_await make_ball(ball, ball_mass, vec3{-sp * m, y, (z0 + zsp * 4) * m},
                           {0.95f, 0.72f, 0.45f});
        co_await make_ball(ball, ball_mass, vec3{0.0 * m, y, (z0 + zsp * 4) * m},
                           {0.0f, 0.5f, 0.1f});
        co_await make_ball(ball, ball_mass, vec3{sp * m, y, (z0 + zsp * 4) * m},
                           {0.55f, 0.75f, 0.55f});
        co_await make_ball(ball, ball_mass, vec3{sp * 2 * m, y, (z0 + zsp * 4) * m},
                           {0.7f, 0.45f, 0.45f});

        co_await add_task<policy::no_wait>(respawn_cue_ball(cue_ball_handle));
        auto shoot_offset = 0.0 * m;
        std::optional<physkit::task_handle> shooting_task;
        while (true)
        {
            auto frame_time = *co_await next_render_frame();
            if ((!shooting_task || !(co_await get_world()).task_active(*shooting_task)) &&
                get_mouse_button(Pointer::MouseLeft).is_initial_press())
                if (auto exp = co_await add_task(
                        charge_and_shoot(stick_handle, anchor_handle, shoot_offset)))
                    shooting_task = *exp;
            update_anchor(frame_time, shoot_offset, stick_obj, anchor_obj);
        }
    }

    task<> charge_and_shoot(world_base::handle stick_handle, world_base::handle anchor_handle,
                            quantity<si::metre> &shoot_offset) // NOLINT
    {
        cam().speed(.1f * si::metre / si::second);
        // Reset velocities and center the stick on the anchor starting a new aim
        auto stick_opt = co_await get_rigid(stick_handle);
        auto anchor_opt = co_await get_rigid(anchor_handle);
        if (stick_opt && anchor_opt)
        {
            stick_opt.value()->vel() = vec3<si::metre / si::second>::zero();
            stick_opt.value()->pos() = anchor_opt.value()->pos();
        }

        // --- Charge phase: Pull the stick backward physically ---
        while (get_mouse_button(Pointer::MouseLeft).is_pressed())
        {
            auto frame_dt = value_cast<float>(*co_await next_frame{});

            auto cur_stick = co_await get_rigid(stick_handle);
            auto cur_anchor = co_await get_rigid(anchor_handle);
            if (cur_stick && cur_anchor)
            {
                auto &stick = **cur_stick;
                auto fwd = cam().forward();

                auto offset = stick.pos() - cur_anchor.value()->pos();
                auto dist = -offset.dot(fwd);

                if (dist < 0.5f * m)
                {
                    auto spring_resistance = 200.0 * kg / s / s * dist;
                    auto damping_resistance = 15.0 * kg / s * stick.vel().dot(-fwd);
                    stick.apply_force(-fwd * (spring_resistance + damping_resistance +
                                              stick.mass() * (25.0 * m / s / s)));
                }
                else
                    stick.vel() *= 0.1;
            }
        }

        // --- Strike phase: Apply a single impulsive strike forward ---
        // The newly added soft spring constraint will catch the stick naturally after it hits
        auto cur_stick = co_await get_rigid(stick_handle);
        auto cur_anchor = co_await get_rigid(anchor_handle);
        if (cur_stick && cur_anchor)
        {
            auto &stick = **cur_stick;
            auto fwd = cam().forward();
            auto offset = stick.pos() - cur_anchor.value()->pos();
            auto pulled_amount = std::max(0.0 * m, -offset.dot(fwd));

            // Shift the anchor forward proportionately so the spring permits a natural
            // follow-through past the ball without violently yanking light taps forward.
            shoot_offset = pulled_amount * 0.75f;

            auto speed = pulled_amount * 10.0 / s;
            stick.vel() = vec3<si::metre / si::second>::zero(); // Stop backing up
            stick.apply_impulse(fwd * stick.mass() * speed);
        }

        // Allow some time for the stick to travel along its stroke
        co_await wait_for{.25 * s};

        // Slowly reel the stick back to its resting state
        while (shoot_offset > 0.0f * m)
        {
            auto frame_dt = value_cast<float>(*co_await next_render_frame());
            shoot_offset = std::max(0.0 * m, shoot_offset - 1.0 * m / s * frame_dt);
        }
        cam().speed(1.0f * si::metre / si::second);
    }
};

MAGNUM_APPLICATION_MAIN(pool_app) // NOLINT