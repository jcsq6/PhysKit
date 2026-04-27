// creating a physics sandbox for contact visualizer

#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif
#ifdef PHYSKIT_MODULES

import physkit;
import mp_units;
#else
#include "sandbox.h"
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

// TODO: Maybe swap to header - driver format and split files
struct sandbox_state
{
    std::optional<world_base::handle> selected;

    std::vector<world_base::handle> dynamic_bodies;

    bool gravity_on = true;

    double debug_timer = 0.0;

    // required state for selection and gravity
    world_base::handle_selected{};
    bool has_selected = false;
    vec3<si::metre / si::second / si::second> saved_gravity = gravity;
}

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
    sandbox_state M_state;

    /// @brief - spawn in different objects
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

    task<> spawn_sphere(vec3<si::metre> pos)
    {
        co_return (*co_await add_rigid(object_desc::dynam()
                                           .with_mesh(mesh::sphere(0.1 * m, 16, 16))
                                           .with_pos(pos)
                                           .with_mass(1.0 * kg)
                                           .with_restitution(0.6)
                                           .with_friction(0.3) Color3{0.8f, 0.8f, 0.8f}))
            ->handle();
    }

    /// TODO: add in different shapes when branches merge - pyramid, cone, etc

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

    /// @brief keyboard and mouse-bindings for the user to manipulate objects in the arena.
    // TODO: Need to bind to other mouse button - or to a keybind since the left click is also drag
    // and view for the world
    task<> maybe_spawn_objects()
    {
        if (get_mouse_button(Pointer::MouseLeft).is_initial_press())
        {
            auto spawn_pos = cam().pos() + cam().forward() * 2.0f * m;
            co_await spawn_box(spawn_pos);
        }
    }

    task<> handle_selection_input()
    {
        if (get_mouse_button(Pointer::MouseLeft).is_initial_press())
        {
            auto ray = physkit::ray{cam().pos(), cam().forward()};

            auto hit = co_await raycast{ray};

            if (hit)
            {
                selected = hit->object;
                has_selected = true;
            }
            else
            {
                has_selected = false;
            }
        }
    }

    /// @brief handles selection, delete, impulse, and velocity reset
    task<> handle_actions_input()
    {
        if (!has_selected) { co_return; }

        auto obj_opt = co_await get_rigid(selected);
        if (!obj_opt)
        {
            has_selected = false;
            co_return;
        }

        auto &obj = **obj_opt;

        // delete function
        if (is_key_pressed(Key::Delete))
        {
            co_await remove_rigid(selected);
            has_selected = false;
        }

        // impulse forward
        if (is_key_pressed(Key::F))
        {
            obj.apply_impulse(cam().forward() * obj.mass() * 5.0 * m / s);
        }
        // Reset velocity
        if (is_key_pressed(Key::R))
        {
            obj.vel() = vec3<si::metre / si::second>::zero();
            obj.ang_vel() = vec3<one / si::second>::zero();
        }

        // Gravity toggle
        if (is_key_pressed(Key::G))
        {
            if (gravity_enabled) { world().gravity(vec3{0.0, 0.0, 0.0} * m / s / s); }
            else
            {
                world().gravity(saved_gravity);
            }

            gravity_enabled = !gravity_enabled;
        }
    }

    /// @brief reset function - resets objects in the arena
    task<> reset_all()
    {
        for (auto h : world().rigid_handles())
        {
            auto obj = co_await get_rigid(h);
            if (obj && obj->is_dynamic())
            {
                obj->vel() = vec3<si::metre / si::second>::zero();
                obj->ang_vel() = vec3<one / si::second>::zero();
            }
        }
    }

    /// @brief keyboard functions for user to select and manipulate objects
    void poll_keys(quantity<si::second> dt)
    {
        if (get_key(Key::One).is_initial_press()) { co_await spawn_box(spawn_pos); }

        if (get_key(Key::Two).is_initial_press()) { co_await spawn_sphere(spawn_pos); }
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

    // build static world geometry, then loop forever on the render frame
    task<> runtime()
    {
        co_await build_area();

        while (true)
        {
            auto dt = *co_await next_render_frame();
            co_await next_render_frame();
            co_await maybe_spawn_objects();
            co_await maybe_pick();
            poll_keys(dt);
        }
    }
};

MAGNUM_APPLICATION_MAIN(sandbox) // NOLINT
