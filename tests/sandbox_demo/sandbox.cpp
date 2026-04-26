// creating a physics sandbox for contact visualizer

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

class sandbox : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world;

public:
    explicit sandbox(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .window_size({1280, 720})
                           .cam_pos(fvec3{0.0f, 2.0f, -3.0f} * si::metre)
                           .look_at(fvec3{0.0f, 0.0f, 1.5f} * si::metre)
                           .drag(false)
                           .gravity(gravity)
                           .time_step(1.0 / 1200.0 * si::second)
                           .solver_iterations(20)}
    {
        cam().speed(1.0f * si::metre / si::second);
        world().add_task(runtime());
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override {}

private:
    task<> spawn_box(vec3<si::metre> pos)
    {
        co_await add_rigid(object_desc::dynam()
                               .with_mesh(mesh::box(vec3{0.2, 0.2, 0.2} * m))
                               .with_pos(pos)
                               .with_mass(1.0 * kg)
                               .with_restitution(0.4)
                               .with_friction(0.5),
                           Color3{0.7f, 0.7f, 0.7f});
    }

    // handle generic spawning inputs
    task<world_base::handle> return_spawn_box(vec3<si::metre> pos)
    {
        co_return (*co_await add_rigid(object_desc::dynam()
                                           .with_mesh(mesh::box(vec3{0.1, 0.1, 0.1} * m))
                                           .with_pos(pos)
                                           .with_mass(1.0 * kg)
                                           .with_restitution(0.4)
                                           .with_friction(0.5),
                                       Color3{0.7f, 0.7f, 0.7f}))
            ->handle();
    }

    // input spawning - based off mouse clicks
    task<> maybe_spawn_objects()
    {
        if (get_mouse_button(Pointer::MouseLeft).is_initial_press())
        {
            auto spawn_pos = cam().pos() + cam().forward() * 2.0f * m;
            co_await spawn_box(spawn_pos);
        }
    }

    task<> build_area()
    {
        auto arena_half = 5.0 * m;
        auto wall_height = 2.0 * m;
        auto thickness = 0.2 * m;

        // Floor
        co_await add_rigid(object_desc::stat()
                               .with_mesh(mesh::box(vec3{arena_half, 0.2 * m, arena_half}))
                               .with_pos(vec3{0.0 * m, -0.2 * m, 0.0 * m})
                               .with_friction(0.8),
                           Color3{0.2f, 0.3f, 0.35f});

        // Back and front walls
        auto wall_fb = mesh::box(vec3{arena_half, wall_height, thickness});

        co_await add_rigid(object_desc::stat()
                               .with_mesh(wall_fb)
                               .with_pos(vec3{0.0 * m, wall_height, -arena_half})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        co_await add_rigid(object_desc::stat()
                               .with_mesh(wall_fb)
                               .with_pos(vec3{0.0 * m, wall_height, arena_half})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        // Left / Right walls
        auto wall_lr = mesh::box(vec3{thickness, wall_height, arena_half});

        co_await add_rigid(object_desc::stat()
                               .with_mesh(wall_lr)
                               .with_pos(vec3{-arena_half, wall_height, 0.0 * m})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        co_await add_rigid(object_desc::stat()
                               .with_mesh(wall_lr)
                               .with_pos(vec3{arena_half, wall_height, 0.0 * m})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        co_return;
    }

    task<> runtime()
    {
        co_await build_area();

        while (true)
        {
            co_await next_render_frame();
            co_await maybe_spawn_objects();
        }
    }
};

MAGNUM_APPLICATION_MAIN(sandbox) // NOLINT
