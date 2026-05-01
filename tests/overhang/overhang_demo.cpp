// Magnum header needed for MAGNUM_APPLICATION_MAIN macro (not exportable from modules)
#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
#include <coroutine> // IWYU pragma: keep
#include <optional>
#include <print>
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

class overhang_app : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world;

    static constexpr auto platform_size = 5.0; // half extent size for platform

    static constexpr auto into_dist = (platform_size); // how far into the platform to do the test.
    // static constexpr auto pivot = vec3{0.0,0.0,platform_size-into_dist}
    static constexpr auto pivot_z = platform_size - into_dist;

    // cards
    static constexpr auto card_hheight =
        0.1 * m; // Only really increases weight. may increase instability?
    static constexpr auto card_hwidth = 0.5 * m;  // Affects calculations
    static constexpr auto card_hlength = 0.2 * m; // Should have no effect
    static constexpr auto card_mass = 1.0 * kg;
    static constexpr std::array colors{Color3{0.5f, 0.0f, 0.0f}, Color3{0.0f, 0.5f, 0.0f},
                                       Color3{0.0f, 0.0f, 0.5f}};

    // static inline const auto card_shape = vec3{1.0, 2.0, 3.0} * m;

public:
    explicit overhang_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, true)
                           .title("Overhang Problem Demo")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{-0.5f, 0.5f, 1.0f + platform_size} * si::metre)
                           .look_at(fvec3{0.0f, 0.0f, 0.0f + pivot_z} * si::metre)
                           .gravity(gravity)
                           .drag(false) // TODO: experiment with these below.
                           .time_step(1.0 / 1500.0 * si::second)
                           .solver_iterations(40)}
    {
        cam().speed(1.0f * si::metre / si::second);
        world().add_task(scene());
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override {}

private:
    static std::vector<quantity<one>> harmonic_numbers;
    // Get nth Harmonic Number
    static quantity<one> harmonic_number(std::size_t n)
    {
        if (harmonic_numbers.empty())
        {
            harmonic_numbers.reserve(30);
            harmonic_numbers.emplace_back(0.0L); // H0
        }

        auto i = harmonic_numbers.size();
        for (; i < n + 1; i++)
        {
            harmonic_numbers.push_back(harmonic_numbers[i - 1] + (1.0 * one) / (i * one));
        }
        return harmonic_numbers[n];
    }

    task<world_base::handle> make_card(std::size_t n)
    {
        auto eps = 0.000 * m; // less than 0.007 lags out.

        auto pos = vec3{eps - harmonic_number(n) * card_hwidth,
                        (2 * n) * -card_hheight + card_hheight, pivot_z * m};
        std::println("New Block {} at: {}", n, pos);
        auto h = (*co_await add_rigid(
                      object_desc::stat() // TODO: not static
                          .with_shape(box(vec3{card_hwidth, card_hheight + eps, card_hlength}))
                          .with_pos(pos)
                          .with_mass(card_mass)
                          .with_restitution(0.0)
                          .with_friction(1),
                      colors[(n) % colors.size()]))
                     ->handle();

        // co_await add_rigid(
        //     object_desc::dynam()
        //         .with_shape(box(vec3{card_hwidth, (card_hheight + eps) * 2.0, card_hlength}))
        //         .with_pos(pos)
        //         .with_mass(card_mass)
        //         .with_restitution(0)
        //         .with_friction(0.7),
        //     colors[(n + 1) % colors.size()]);
        // co_await add_task<policy::no_wait>(
        //     [h](this auto self) -> task<> // NOLINT
        //     {
        //         while (auto opt = co_await get_rigid(h))
        //         {
        //             co_await next_frame{};
        //         }
        //     }());
        co_return h;
    }

    task<> scene()
    {
        auto eps = 0.001;
        auto initial_pos = vec3{(eps - platform_size) * m, -platform_size * m, 0.0 * m};
        auto platform =
            (*co_await add_rigid(
                 object_desc::stat()
                     .with_shape(box(vec3{platform_size, platform_size, platform_size} * m))
                     .with_pos(initial_pos)
                     .with_restitution(0.0)
                     .with_friction(1),
                 Color3{0.1f, 0.45f, 0.15f}))
                ->handle();
        auto &plat_pos = (*co_await get_rigid(platform))->pos();
        // auto h = *co_await make_card(0);

        auto cam_pos = cam().pos();
        auto &track = cam().move_track();

        auto count = 0;
        while (true)
        {
            auto frame_time = *co_await next_render_frame();
            auto h = *co_await make_card(count);

            co_await wait_for(4 * s);
            auto offset = harmonic_number(count) * card_hwidth;
            plat_pos =
                vec3{plat_pos.x() /*- offset*/, plat_pos.y() - 2.0 * card_hheight, plat_pos.z()};
            cam().move(fvec3{0.0f * m, //-static_cast<quantity<m, float>>(offset),
                             (-2.0f * static_cast<quantity<m, float>>(card_hheight)), 0.0f * m});

            // if ((!shooting_task || !(co_await get_world()).task_active(*shooting_task)) &&
            //     //get_mouse_button(Pointer::MouseLeft).is_initial_press())
            //     if (auto exp = co_await add_task(
            //             charge_and_shoot(stick_handle, anchor_handle, shoot_offset)))
            //         shooting_task = *exp;
            // update_anchor(frame_time, shoot_offset, stick_obj, anchor_obj);

            count++;
        }
    }
    // // --- Anchor (invisible static body that drives the stick via weld constraint) ---
    // auto initial_pos = vec3{0.0, 5.0, 0.0} * m;
    // auto anchor_handle =
    //     *co_await create_rigid(object_desc::stat()
    //                                .with_shape(box(vec3{0.001, 0.001, 0.001} * m))
    //                                .with_pos(initial_pos));

    // // --- Stick ---
    // auto stick_handle = (*co_await add_rigid(object_desc::dynam()
    //                                              .with_shape(box(vec3{0.005, 0.75, 0.005} * m))
    //                                              .with_pos(initial_pos)
    //                                              .with_mass(.5 * kg)
    //                                              .with_restitution(1)
    //                                              .with_friction(0.05),
    //                                          Color3{0.72f, 0.53f, 0.2f}))
    //                         ->handle();

    // // --- Constraint ---
    // auto &anchor_obj = **co_await get_rigid(anchor_handle);
    // auto &stick_obj = **co_await get_rigid(stick_handle);

    // // Slider keeps the stick structurally aligned with the camera vector (Z-axis local)
    // co_await physkit::add_constraint{
    //     impulse::slider_constraint::desc::make(anchor_handle, stick_handle)
    //         .with_anchor(initial_pos)
    //         .with_axis(vec3<one>{0.0, 1.0, 0.0})};

    // // Soft spring keeps it tethered elastically, pulling the stick through dynamic physics
    // co_await physkit::add_constraint{
    //     impulse::spring_constraint::desc::make(anchor_handle, stick_handle)
    //         .with_local_anchor_a(anchor_obj.project_to_local(initial_pos))
    //         .with_local_anchor_b(stick_obj.project_to_local(initial_pos))
    //         .with_distance(0.0 * m)
    //         .with_stiffness(200.0 * kg / s / s)
    //         .with_damping(15.0 * kg / s)};

    // // --- Table ---
    // auto felt_mesh = box(vec3{0.8, 0.1, 1.6} * m);
    // auto felt_pos = vec3{0.0, -0.1, 0.0} * m;

    // co_await add_rigid(object_desc::stat()
    //                        .with_shape(felt_mesh)
    //                        .with_pos(felt_pos)
    //                        .with_restitution(0.3)
    //                        .with_friction(1),
    //                    Color3{0.1f, 0.45f, 0.15f});

    // auto felt_bounds = felt_mesh.bounds();
    // auto felt_hx = (felt_bounds.max.x() - felt_bounds.min.x()) * 0.5;
    // auto felt_hz = (felt_bounds.max.z() - felt_bounds.min.z()) * 0.5;

    // auto rail_t = 0.05 * m;   // rail half-thickness
    // auto rail_h = 0.05 * m;   // rail half-height
    // auto pocket_r = 0.07 * m; // size of the pocket gaps

    // // Top of felt is at y = 0.0m
    // // We center rails at y = 0.0m so their tops are at 0.05m
    // auto rail_fb_hx = felt_hx - pocket_r;
    // auto rail_fb = box(vec3{rail_fb_hx, rail_h, rail_t});

    // auto rail_lr_hz = (felt_hz - 2.0 * pocket_r) * 0.5;
    // auto rail_lr = box(vec3{rail_t, rail_h, rail_lr_hz});
    // auto lr_z_offset = pocket_r + rail_lr_hz;

    // Color3 wood{0.45f, 0.25f, 0.1f};

    // // co_await make_rail(rail_fb, vec3{0.0 * m, 0.0 * m, -felt_hz - rail_t}, wood);
    // // co_await make_rail(rail_fb, vec3{0.0 * m, 0.0 * m, felt_hz + rail_t}, wood);

    // // co_await make_rail(rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, -lr_z_offset}, wood);
    // // co_await make_rail(rail_lr, vec3{-felt_hx - rail_t, 0.0 * m, lr_z_offset}, wood);
    // // co_await make_rail(rail_lr, vec3{felt_hx + rail_t, 0.0 * m, -lr_z_offset}, wood);
    // // co_await make_rail(rail_lr, vec3{felt_hx + rail_t, 0.0 * m, lr_z_offset}, wood);

    // // --- Balls ---

    // // Cue ball
    // auto sp = ball_radius * 2.0 + 0.001 * m; // just over diameter

    // auto cue_ball_handle =
    //     *co_await make_ball(vec3{0.0 * m, ball_radius, -0.8 * m}, {0.95f, 0.95f, 0.95f});

    // // Rack: 5 rows, apex at z=0.8
    // auto zsp = sp * 0.866; // equilateral triangle row spacing
    // auto z0 = 0.8 * m;
    // auto y = ball_radius;

    // // Row 1
    // co_await make_ball(vec3{0.0 * m, y, z0}, {1.0f, 0.85f, 0.0f});
    // // Row 2
    // co_await make_ball(vec3{-sp / 2, y, (z0 + zsp)}, {1.0f, 0.92f, 0.55f});
    // co_await make_ball(vec3{sp / 2, y, (z0 + zsp)}, {0.0f, 0.25f, 0.85f});
    // // Row 3
    // co_await make_ball(vec3{-sp, y, (z0 + zsp * 2)}, {0.55f, 0.65f, 0.9f});
    // co_await make_ball(vec3{0.0 * m, y, (z0 + zsp * 2)}, {0.08f, 0.08f, 0.08f}); // 8-ball
    // co_await make_ball(vec3{sp, y, (z0 + zsp * 2)}, {0.85f, 0.1f, 0.1f});
    // // Row 4
    // co_await make_ball(vec3{-sp * 1.5, y, (z0 + zsp * 3)}, {0.9f, 0.55f, 0.55f});
    // co_await make_ball(vec3{-sp / 2, y, (z0 + zsp * 3)}, {0.5f, 0.1f, 0.1f});
    // co_await make_ball(vec3{sp / 2, y, (z0 + zsp * 3)}, {0.5f, 0.0f, 0.5f});
    // co_await make_ball(vec3{sp * 1.5, y, (z0 + zsp * 3)}, {0.7f, 0.45f, 0.7f});
    // // Row 5
    // co_await make_ball(vec3{-sp * 2, y, (z0 + zsp * 4)}, {0.95f, 0.5f, 0.0f});
    // co_await make_ball(vec3{-sp, y, (z0 + zsp * 4)}, {0.95f, 0.72f, 0.45f});
    // co_await make_ball(vec3{0.0 * m, y, (z0 + zsp * 4)}, {0.0f, 0.5f, 0.1f});
    // co_await make_ball(vec3{sp, y, (z0 + zsp * 4)}, {0.55f, 0.75f, 0.55f});
    // co_await make_ball(vec3{sp * 2, y, (z0 + zsp * 4)}, {0.7f, 0.45f, 0.45f});

    // co_await add_task<policy::no_wait>(respawn_cue_ball(cue_ball_handle));
    // auto shoot_offset = 0.0 * m;
    // std::optional<physkit::task_handle> shooting_task;
    // while (true)
    // {
    //     auto frame_time = *co_await next_render_frame();
    //     if ((!shooting_task || !(co_await get_world()).task_active(*shooting_task)) &&
    //         get_mouse_button(Pointer::MouseLeft).is_initial_press())
    //         if (auto exp = co_await add_task(
    //                 charge_and_shoot(stick_handle, anchor_handle, shoot_offset)))
    //             shooting_task = *exp;
    //     update_anchor(frame_time, shoot_offset, stick_obj, anchor_obj);
    // }
};
std::vector<quantity<one>> overhang_app::harmonic_numbers;

MAGNUM_APPLICATION_MAIN(overhang_app) // NOLINT