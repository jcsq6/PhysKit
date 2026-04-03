// Magnum header needed for MAGNUM_APPLICATION_MAIN macro (not exportable from modules)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
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
    using typed_world = physkit::world<physkit::semi_implicit_euler>;

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
        world().add_task(scene(this));
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override
    {
        update_anchor(dt);

        if (!M_cue_ball_handle.is_null_handle())
        {
            if (auto cue_ball_opt = world().get_rigid(M_cue_ball_handle))
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

    void pointer_press(PointerEvent &event, bool pressed) override
    {
        if (event.pointer() == Pointer::MouseLeft)
        {
            if (pressed)
                world().add_task(charge_and_shoot(this));
            else
                M_released = true;
        }
    }

private:
    world_base::handle M_anchor_handle = world_base::handle::from_id(world_base::handle::null);
    world_base::handle M_stick_handle = world_base::handle::from_id(world_base::handle::null);
    world_base::handle M_cue_ball_handle = world_base::handle::from_id(world_base::handle::null);
    quantity<si::metre, float> M_pull_back = 0.0f * si::metre;
    quantity<si::metre, float> M_shoot_offset = 0.0f * si::metre;
    bool M_released{false};

    // Teleport the invisible anchor to the desired camera-relative position.
    // The stick follows it kinematically during aiming.
    void update_anchor(mp_units::quantity<mp_units::si::second> dt)
    {
        if (M_anchor_handle.is_null_handle()) return;
        auto anchor_opt = world().get_rigid(M_anchor_handle);
        if (!anchor_opt) return;
        auto &anchor = **anchor_opt;

        auto stick_opt = world().get_rigid(M_stick_handle);
        if (!stick_opt) return;
        auto &stick = **stick_opt;
        auto bounds = stick.mesh().bounds();
        auto stick_half_len = (bounds.max.y() - bounds.min.y()) * 0.5f;

        auto fwd = cam().forward();
        auto tip_z = 1.0f * m; // Keep resting position anchor stationary
        auto stick_half_len_float = value_cast<float>(stick_half_len);
        auto target_pos = cam().pos() +
                          fwd * (tip_z + value_cast<float>(M_shoot_offset) - stick_half_len_float) +
                          cam().right() * 0.10f * m + cam().up() * -0.05f * m;

        auto magnum_fwd = to_magnum_vector<one, float>(fwd);
        auto target_rot =
            to_physkit_quaternion<one, double>(Quaternion::rotation(Vector3::yAxis(), -magnum_fwd));

        anchor.pos() = target_pos;
        anchor.orientation(target_rot);
    }

    static task<> make_rail(pool_app *self, std::shared_ptr<physkit::mesh> msh, vec3<si::metre> pos,
                            Color3 color)
    {
        auto rh = *co_await create_rigid(
            object_desc::stat().with_mesh(msh).with_pos(pos).with_restitution(0.75).with_friction(
                0.3));
        self->add_object(rh, color);
    }

    static task<world_base::handle> make_ball(pool_app *self, std::shared_ptr<physkit::mesh> ball,
                                              physkit::quantity<si::kilogram, double> ball_mass,
                                              vec3<si::metre> pos, Color3 color)
    {
        auto bh = *co_await create_rigid(object_desc::dynam()
                                             .with_mesh(ball)
                                             .with_pos(pos)
                                             .with_mass(ball_mass)
                                             .with_restitution(0.9)
                                             .with_friction(0.005));
        self->add_object(bh, color);
        co_return bh;
    }

    static task<> scene(pool_app *self)
    {
        // --- Anchor (invisible static body that drives the stick via weld constraint) ---
        auto initial_pos = vec3{0.0, 5.0, 0.0} * m;
        self->M_anchor_handle =
            *co_await create_rigid(object_desc::stat()
                                       .with_mesh(mesh::box(vec3{0.001, 0.001, 0.001} * m))
                                       .with_pos(initial_pos));

        // --- Stick ---
        self->M_stick_handle =
            *co_await create_rigid(object_desc::dynam()
                                       .with_mesh(mesh::box(vec3{0.005, 0.75, 0.005} * m))
                                       .with_pos(initial_pos)
                                       .with_mass(.5 * kg)
                                       .with_restitution(1)
                                       .with_friction(0.05));
        self->add_object(self->M_stick_handle, Color3{0.72f, 0.53f, 0.2f});

        // --- Constraint ---
        auto &anchor_obj = **self->world().get_rigid(self->M_anchor_handle);
        auto &stick_obj = **self->world().get_rigid(self->M_stick_handle);
        auto &w = dynamic_cast<typed_world &>(self->world());

        // Slider keeps the stick structurally aligned with the camera vector (Z-axis local)
        w.add_constraint(impulse::slider_constraint{anchor_obj, stick_obj, initial_pos,
                                                    vec3<one>{0.0, 1.0, 0.0}});

        // Soft spring keeps it tethered elastically, pulling the stick through dynamic physics
        w.add_constraint(impulse::spring_constraint{
            anchor_obj, stick_obj, anchor_obj.project_to_local(initial_pos),
            stick_obj.project_to_local(initial_pos), 0.0 * m,
            200.0 * kg / s / s, // stiffness (N/m)
            15.0 * kg / s       // damping (N*s/m)
        });

        // --- Table ---
        auto felt_mesh = mesh::box(vec3{0.8, 0.1, 1.6} * m);
        auto felt_pos = vec3{0.0, -0.1, 0.0} * m;

        auto felt = *co_await create_rigid(object_desc::stat()
                                               .with_mesh(felt_mesh)
                                               .with_pos(felt_pos)
                                               .with_restitution(0.3)
                                               .with_friction(0.6));
        self->add_object(felt, Color3{0.1f, 0.45f, 0.15f});

        auto felt_bounds = felt_mesh->bounds();
        auto felt_hx = (felt_bounds.max.x() - felt_bounds.min.x()) * 0.5;
        auto felt_hz = (felt_bounds.max.z() - felt_bounds.min.z()) * 0.5;

        auto rail_t = 0.05 * m;   // rail half-thickness
        auto rail_h = 0.05 * m;   // rail half-height
        auto pocket_r = 0.07 * m; // size of the pocket gaps

        // Top of felt is at y = 0.0m
        // We center rails at y = 0.0m so their tops are at 0.05m
        auto rail_fb_hx = felt_hx - pocket_r;
        auto rail_fb = mesh::box(vec3{rail_fb_hx, rail_h, rail_t});

        auto rail_lr_hz = (felt_hz - 2.0 * pocket_r) * 0.5;
        auto rail_lr = mesh::box(vec3{rail_t, rail_h, rail_lr_hz});
        auto lr_z_offset = pocket_r + rail_lr_hz;

        Color3 wood{0.45f, 0.25f, 0.1f};

        co_await make_rail(self, rail_fb, vec3{0.0 * m, 0.0 * m, -felt_hz - rail_t}, wood);
        co_await make_rail(self, rail_fb, vec3{0.0 * m, 0.0 * m, felt_hz + rail_t}, wood);

        co_await make_rail(self, rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, -lr_z_offset}, wood);
        co_await make_rail(self, rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, lr_z_offset}, wood);
        co_await make_rail(self, rail_lr, vec3{felt_hx + rail_t, 0.0 * m, -lr_z_offset}, wood);
        co_await make_rail(self, rail_lr, vec3{felt_hx + rail_t, 0.0 * m, lr_z_offset}, wood);

        // --- Balls ---
        auto ball = mesh::sphere(0.0285 * m, 16, 24);
        auto ball_mass = 0.17 * kg;

        // Cue ball
        auto ball_bounds = ball->bounds();
        auto ball_radius = (ball_bounds.max.x() - ball_bounds.min.x()) * 0.5;
        double sp = (ball_bounds.max.x() - ball_bounds.min.x()).numerical_value_in(m) +
                    0.001; // just over diameter

        self->M_cue_ball_handle = *co_await make_ball(
            self, ball, ball_mass, vec3{0.0 * m, ball_radius, -0.8 * m}, {0.95f, 0.95f, 0.95f});

        // Rack: 5 rows, apex at z=0.8
        double zsp = sp * 0.866; // equilateral triangle row spacing
        double z0 = 0.8;
        auto y = ball_radius;

        // Row 1
        co_await make_ball(self, ball, ball_mass, vec3{0.0 * m, y, z0 * m}, {1.0f, 0.85f, 0.0f});
        // Row 2
        co_await make_ball(self, ball, ball_mass, vec3{-sp / 2 * m, y, (z0 + zsp) * m},
                           {1.0f, 0.92f, 0.55f});
        co_await make_ball(self, ball, ball_mass, vec3{sp / 2 * m, y, (z0 + zsp) * m},
                           {0.0f, 0.25f, 0.85f});
        // Row 3
        co_await make_ball(self, ball, ball_mass, vec3{-sp * m, y, (z0 + zsp * 2) * m},
                           {0.55f, 0.65f, 0.9f});
        co_await make_ball(self, ball, ball_mass, vec3{0.0 * m, y, (z0 + zsp * 2) * m},
                           {0.08f, 0.08f, 0.08f}); // 8-ball
        co_await make_ball(self, ball, ball_mass, vec3{sp * m, y, (z0 + zsp * 2) * m},
                           {0.85f, 0.1f, 0.1f});
        // Row 4
        co_await make_ball(self, ball, ball_mass, vec3{-sp * 1.5 * m, y, (z0 + zsp * 3) * m},
                           {0.9f, 0.55f, 0.55f});
        co_await make_ball(self, ball, ball_mass, vec3{-sp / 2 * m, y, (z0 + zsp * 3) * m},
                           {0.5f, 0.1f, 0.1f});
        co_await make_ball(self, ball, ball_mass, vec3{sp / 2 * m, y, (z0 + zsp * 3) * m},
                           {0.5f, 0.0f, 0.5f});
        co_await make_ball(self, ball, ball_mass, vec3{sp * 1.5 * m, y, (z0 + zsp * 3) * m},
                           {0.7f, 0.45f, 0.7f});
        // Row 5
        co_await make_ball(self, ball, ball_mass, vec3{-sp * 2 * m, y, (z0 + zsp * 4) * m},
                           {0.95f, 0.5f, 0.0f});
        co_await make_ball(self, ball, ball_mass, vec3{-sp * m, y, (z0 + zsp * 4) * m},
                           {0.95f, 0.72f, 0.45f});
        co_await make_ball(self, ball, ball_mass, vec3{0.0 * m, y, (z0 + zsp * 4) * m},
                           {0.0f, 0.5f, 0.1f});
        co_await make_ball(self, ball, ball_mass, vec3{sp * m, y, (z0 + zsp * 4) * m},
                           {0.55f, 0.75f, 0.55f});
        co_await make_ball(self, ball, ball_mass, vec3{sp * 2 * m, y, (z0 + zsp * 4) * m},
                           {0.7f, 0.45f, 0.45f});
    }

    static task<> charge_and_shoot(pool_app *self)
    {
        self->M_released = false;

        // Reset velocities and center the stick on the anchor starting a new aim
        auto stick_opt = self->world().get_rigid(self->M_stick_handle);
        auto anchor_opt = self->world().get_rigid(self->M_anchor_handle);
        if (stick_opt && anchor_opt)
        {
            stick_opt.value()->vel() = vec3<si::metre / si::second>::zero();
            stick_opt.value()->pos() = anchor_opt.value()->pos();
        }

        // --- Charge phase: Pull the stick backward physically ---
        while (!self->M_released)
        {
            auto frame_dt = value_cast<float>(*co_await next_frame{});
            auto cur_stick = self->world().get_rigid(self->M_stick_handle);
            auto cur_anchor = self->world().get_rigid(self->M_anchor_handle);
            if (cur_stick && cur_anchor)
            {
                auto &stick = **cur_stick;
                auto fwd = self->cam().forward();

                auto offset = stick.pos() - cur_anchor.value()->pos();
                auto dist = -offset.dot(fwd);

                // Pull back smoothly, limit max pull to ~0.5m for a realistic cue stroke
                if (dist < 0.5f * m)
                {
                    // The spring pulls forward with 200 N/m. We must overcome it to pull the stick
                    // back! Overcome the damping as well.
                    auto spring_resistance = 200.0 * kg / s / s * dist;
                    auto damping_resistance = 15.0 * kg / s * stick.vel().dot(-fwd);
                    // Apply enough force to perfectly counter the spring, plus enough to accelerate
                    // backward slowly
                    stick.apply_force(-fwd * (spring_resistance + damping_resistance +
                                              stick.mass() * (25.0 * m / s / s)));
                }
                else
                {
                    stick.vel() *= 0.1; // Dampen heavily at the max pull limit to hold it steady
                }
            }
        }

        // --- Strike phase: Apply a single impulsive strike forward ---
        // The newly added soft spring constraint will catch the stick naturally after it hits
        auto cur_stick = self->world().get_rigid(self->M_stick_handle);
        auto cur_anchor = self->world().get_rigid(self->M_anchor_handle);
        if (cur_stick && cur_anchor)
        {
            auto &stick = **cur_stick;
            auto fwd = self->cam().forward();
            auto offset = stick.pos() - cur_anchor.value()->pos();
            auto pulled_amount = std::max(0.0 * m, -offset.dot(fwd));

            // Shift the anchor forward proportionately so the spring permits a natural
            // follow-through past the ball without violently yanking light taps forward.
            self->M_shoot_offset = pulled_amount * 0.75f;

            auto speed = pulled_amount * 10.0 / s;
            stick.vel() = vec3<si::metre / si::second>::zero(); // Stop backing up
            stick.apply_impulse(fwd * stick.mass() * speed);
        }

        // Allow some time for the stick to travel along its stroke
        auto t = 0.0f * s;
        while (t < 0.25f * s)
        {
            auto frame_dt = value_cast<float>(*co_await next_frame{});
            t += frame_dt;
        }

        // Slowly reel the stick back to its resting state
        while (self->M_shoot_offset > 0.0f * m)
        {
            auto frame_dt = value_cast<float>(*co_await next_frame{});
            self->M_shoot_offset =
                std::max(0.0f * m, self->M_shoot_offset - 1.0f * m / s * frame_dt);
        }
    }
};

MAGNUM_APPLICATION_MAIN(pool_app) // NOLINT