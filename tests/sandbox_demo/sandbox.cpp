// creating a physics sandbox for contact visualizer

#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
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

// TODO: Maybe swap to header - driver format and split files
struct sandbox_state
{
    std::optional<world_base::handle> selected;
    bool gravity_on = true;
    vec3<si::metre / si::second / si::second> saved_gravity{};
};

class sandbox : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world;

    // TODO: test other screen resolutions - perhaps sandbox demo should be fullscreen in the
    // future.
public:
    explicit sandbox(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .window_size({1600, 900})
                           .cam_pos(fvec3{22.0f, 16.0f, -28.0f} * si::metre)
                           .look_at(fvec3{-2.0f, 3.0f, 0.0f} * si::metre)
                           .drag(false)
                           .crosshair_overlay()
                           .gravity(gravity)
                           .time_step(1.0 / 1200.0 * si::second)
                           .solver_iterations(32)}
    {
        cam().speed(3.0f * si::metre / si::second);
        M_state.saved_gravity = gravity;
        debug_overlay().controls("Controls", {
                                                 "WASD  move camera",
                                                 "Space / Left Shift  up / down",
                                                 "Mouse  look",
                                                 "Esc  release or capture mouse",
                                                 "LMB  select dynamic object",
                                                 "RMB or 1  spawn box",
                                                 "2  spawn sphere",
                                                 "F  impulse selected object",
                                                 "Delete  delete selected object",
                                                 "G  toggle gravity",
                                                 "R  reset velocities",
                                                 "F9  debug overlay",
                                             });
        world().add_task(runtime());
        // auto &w = dynamic_cast<physkit::world<physkit::semi_implicit_euler> &>(world());
    }

    void update(mp_units::quantity<mp_units::si::second> /*dt*/) override {}

private:
    sandbox_state M_state;

    [[nodiscard]] const auto &world_gravity() const
    {
        const auto &const_world = static_cast<const world_base &>(world());
        return const_world.gravity();
    }

    // graphics_app only exposes a const gravity accessor, but the sandbox needs
    // to toggle it at runtime.
    auto &mutable_world_gravity()
    { return const_cast<vec3<si::metre / si::second / si::second> &>(world_gravity()); }

    void remove_physics_object(world_base::handle handle)
    {
        physics_obj *to_remove = nullptr;
        for (auto *obj : physics_objects())
        {
            if (obj->handle() == handle)
            {
                to_remove = obj;
                break;
            }
        }

        delete to_remove;
    }

    /// @brief - spawn in different objects
    /// do deliberate pass of vol, density, inertia
    task<> spawn_box(vec3<si::metre> pos)
    {
        auto shp = physkit::shape{box(vec3{0.2, 0.2, 0.2} * m)};
        auto mass = 1.0 * kg;
        auto density = mass / shp.volume();
        auto inertia = shp.inertia_tensor(density);

        co_await add_rigid(object_desc::dynam()
                               .with_shape(shp)
                               .with_pos(pos)
                               .with_mass(mass)
                               .with_inertia_tensor(inertia)
                               .with_ang_vel(vec3{4.0, 8.0, 2.0} * rad / s)
                               .with_restitution(0.4)
                               .with_friction(0.5),
                           Color3{0.7f, 0.7f, 0.7f});
        co_return;

        // you could optionally do it this way
        /*co_await add_rigid(object_desc::dynam()
                               .with_shape(box(vec3{0.2, 0.2, 0.2} * m))
                               .with_pos(pos)
                               .with_mass(1.0 * kg)
                               .with_ang_vel(vec3{4.0, 8.0, 2.0} * rad / s)
                               .with_restitution(0.4)
                               .with_friction(0.5),
                           Color3{0.7f, 0.7f, 0.7f});
        co_return;*/
    }

    task<> spawn_sphere(vec3<si::metre> pos)
    {
        auto shp = physkit::shape{sphere(0.1 * m)};
        auto mass = 1.0 * kg;
        auto density = mass / shp.volume();
        auto inertia = shp.inertia_tensor(density);

        co_await add_rigid(object_desc::dynam()
                               .with_shape(shp)
                               .with_pos(pos)
                               .with_mass(mass)
                               .with_inertia_tensor(inertia)
                               .with_ang_vel(vec3{20, 0, 0} * rad / s)
                               .with_restitution(0.6)
                               .with_friction(0.3),
                           Color3{0.8f, 0.8f, 0.8f});
        co_return;

        // see above
        /*co_await add_rigid(object_desc::dynam()
                               .with_shape(sphere(0.1 * m))
                               .with_pos(pos)
                               .with_mass(1.0 * kg)
                               .with_restitution(0.6)
                               .with_friction(0.3),
                           Color3{0.8f, 0.8f, 0.8f});
        co_return;*/
    }

    /// TODO: add in different shapes when branches merge - pyramid, cone, etc

    /// @brief keyboard and mouse-bindings for the user to manipulate objects in the arena.
    task<> maybe_spawn_objects()
    {
        if (get_mouse_button(Pointer::MouseRight).is_initial_press())
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
            auto hits = co_await raycast{.r = ray};
            M_state.selected.reset();

            for (auto hit : hits)
            {
                auto obj_opt = co_await get_rigid(hit.first);
                if (obj_opt && (*obj_opt)->is_dynamic())
                {
                    M_state.selected = hit.first;
                    break;
                }
            }
        }
    }

    /// @brief handles selection, delete, impulse, and velocity reset
    task<> handle_actions_input()
    {
        if (get_key(Key::R).is_initial_press()) { co_await reset_all(); }

        if (get_key(Key::G).is_initial_press())
        {
            if (M_state.gravity_on)
                mutable_world_gravity() = vec3{0.0, 0.0, 0.0} * m / s / s;
            else
                mutable_world_gravity() = M_state.saved_gravity;

            M_state.gravity_on = !M_state.gravity_on;
        }

        if (!M_state.selected) { co_return; }

        auto selected = *M_state.selected;
        auto obj_opt = co_await get_rigid(selected);
        if (!obj_opt)
        {
            M_state.selected.reset();
            co_return;
        }

        auto &obj = **obj_opt;

        // delete function
        if (get_key(Key::Delete).is_initial_press())
        {
            co_await destroy_rigid{selected};
            remove_physics_object(selected);
            M_state.selected.reset();
            co_return;
        }

        // impulse forward
        if (get_key(Key::F).is_initial_press())
        {
            obj.apply_impulse(cam().forward() * obj.mass() * 5.0 * m / s);
        }
    }

    /// @brief reset function - resets objects in the arena
    task<> reset_all()
    {
        for (auto *phys_obj : physics_objects())
        {
            auto obj = co_await get_rigid(phys_obj->handle());
            if (obj && (*obj)->is_dynamic())
            {
                (*obj)->vel() = vec3<si::metre / si::second>::zero();
                (*obj)->ang_vel() = vec3<one / si::second>::zero();
            }
        }
    }

    /// @brief keyboard functions for user to select and manipulate objects
    task<> poll_keys(vec3<si::metre> spawn_pos)
    {
        if (get_key(Key::One).is_initial_press()) { co_await spawn_box(spawn_pos); }

        if (get_key(Key::Two).is_initial_press()) { co_await spawn_sphere(spawn_pos); }
    }

    task<> build_area()
    {
        auto arena_half = 30.0 * m;
        auto wall_height = 5.0 * m;
        auto thickness = 0.4 * m;

        // Floor
        co_await add_rigid(object_desc::stat()
                               .with_shape(box(vec3{arena_half, 0.2 * m, arena_half}))
                               .with_pos(vec3{0.0 * m, -0.2 * m, 0.0 * m})
                               .with_friction(0.8),
                           Color3{0.2f, 0.3f, 0.35f});

        // Back and front walls
        auto wall_fb = box(vec3{arena_half, wall_height, thickness});

        co_await add_rigid(object_desc::stat()
                               .with_shape(wall_fb)
                               .with_pos(vec3{0.0 * m, wall_height, -arena_half})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        co_await add_rigid(object_desc::stat()
                               .with_shape(wall_fb)
                               .with_pos(vec3{0.0 * m, wall_height, arena_half})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        // Left / Right walls
        auto wall_lr = box(vec3{thickness, wall_height, arena_half});

        co_await add_rigid(object_desc::stat()
                               .with_shape(wall_lr)
                               .with_pos(vec3{-arena_half, wall_height, 0.0 * m})
                               .with_friction(0.7),
                           Color3{0.4f, 0.4f, 0.4f});

        co_await add_rigid(object_desc::stat()
                               .with_shape(wall_lr)
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
            co_await next_render_frame();
            co_await maybe_spawn_objects();
            co_await handle_selection_input();
            co_await handle_actions_input();

            auto spawn_pos = cam().pos() + cam().forward() * 2.0f * m;
            co_await poll_keys(spawn_pos);
        }
    }
};

MAGNUM_APPLICATION_MAIN(sandbox) // NOLINT
