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

    static constexpr auto eps = 0.1;

    // cards
    static constexpr auto card_hheight =
        0.1 * m; // Only really increases weight. may increase instability?
    static constexpr auto card_hwidth = 0.4 * m;  // Affects calculations
    static constexpr auto card_hlength = 0.2 * m; // Should have no effect
    static constexpr auto card_mass = 1.0 * kg;
    static constexpr std::array colors{
        Color3{0.75f, 0.25f, 0.25f}, // Red
        Color3{0.75f, 0.50f, 0.25f}, // Orange
        Color3{0.75f, 0.75f, 0.25f}, // Yellow
        Color3{0.25f, 0.65f, 0.25f}, // Green
        Color3{0.25f, 0.65f, 0.65f}, // Cyan
        Color3{0.25f, 0.25f, 0.75f}, // Blue
        Color3{0.45f, 0.25f, 0.65f}, // Purple
        Color3{0.75f, 0.25f, 0.65f}  // Magenta
    };

    // static inline const auto card_shape = vec3{1.0, 2.0, 3.0} * m;

public:
    explicit overhang_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, true)
                           .title("Overhang Problem Demo")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{2.0f, 0.8f, 3.0f + platform_size} * si::metre)
                           .look_at(fvec3{0.5f, 1.0f, 0.0f + pivot_z} * si::metre)
                           .lights(std::vector<Vector4>{{-10.f, 50.f, 30.f, 0.f}})
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
        auto pos = vec3{-(1.0 - eps) * (harmonic_number(n)) * card_hwidth,
                        (2 * n) * -card_hheight + card_hheight, pivot_z * m};
        std::println("New Block {} at: {}", n, pos);
        auto h =
            (*co_await add_rigid(object_desc::stat() // TODO: not static
                                     .with_shape(box(vec3{card_hwidth, card_hheight, card_hlength}))
                                     .with_pos(pos)
                                     .with_mass(card_mass)
                                     .with_restitution(0.0)
                                     .with_friction(1),
                                 colors[(n) % colors.size()]))
                ->handle();
        co_return h;
    }

    task<> scene()
    {
        auto initial_pos = vec3{card_hwidth + (-platform_size) * m,
                                (2.0) * card_hheight - platform_size * m, 0.0 * m};
        auto platform =
            (*co_await add_rigid(
                 object_desc::stat()
                     .with_shape(box(vec3{platform_size, platform_size, platform_size} * m))
                     .with_pos(initial_pos)
                     .with_restitution(0.0)
                     .with_friction(1),
                 Color3{0.4f, 0.4f, 0.4f}))
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
            auto offset = (1.0 - eps) * harmonic_number(count + 1) * card_hwidth;
            plat_pos =
                vec3{initial_pos.x() - offset, plat_pos.y() - 2.0 * card_hheight, plat_pos.z()};
            cam().move(fvec3{-static_cast<quantity<m, float>>((1.0 / (count + 1)) * card_hwidth),
                             (-2.0f * static_cast<quantity<m, float>>(card_hheight)), 0.0f * m});

            co_await wait_for(3 * s);

            count++;
        }
    }
};
std::vector<quantity<one>> overhang_app::harmonic_numbers;

MAGNUM_APPLICATION_MAIN(overhang_app) // NOLINT