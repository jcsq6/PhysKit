// creating a physics sandbox for contact visualizer

#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <coroutine> // IWYU pragma: keep
#include <memory>
#include <optional>
#include <string>
#include <vector>
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
    std::vector<world_base::handle> frozen;
    unsigned spawn_shape_index = 0;
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
        cam().speed(10.0f * si::metre / si::second);
        M_state.saved_gravity = gravity;
        refresh_controls();
        world().add_task(runtime());
        // auto &w = dynamic_cast<physkit::world<physkit::semi_implicit_euler> &>(world());
    }

    void update(mp_units::quantity<mp_units::si::second> /*dt*/) override {}

private:
    static constexpr unsigned spawn_shape_count = 5;
    sandbox_state M_state;

    void scrollEvent(Platform::Application::ScrollEvent &event) override
    {
        if (event.offset().y() > 0.0f)
            select_spawn_shape(1);
        else if (event.offset().y() < 0.0f)
            select_spawn_shape(-1);
    }

    static const char *spawn_shape_name(unsigned index)
    {
        switch (index % spawn_shape_count)
        {
        case 0:
            return "box";
        case 1:
            return "sphere";
        case 2:
            return "cylinder";
        case 3:
            return "cone";
        case 4:
            return "pyramid";
        default:
            return "box";
        }
    }

    static shape spawn_shape(unsigned index)
    {
        switch (index % spawn_shape_count)
        {
        case 0:
            return box(vec3{0.2, 0.2, 0.2} * m);
        case 1:
            return sphere(0.18 * m);
        case 2:
            return cylinder(0.18 * m, 0.4 * m);
        case 3:
            return cone(0.2 * m, 0.4 * m);
        case 4:
            return pyramid(0.2 * m, 0.4 * m);
        default:
            return box(vec3{0.2, 0.2, 0.2} * m);
        }
    }

    static Color3 spawn_shape_color(unsigned index)
    {
        switch (index % spawn_shape_count)
        {
        case 0:
            return Color3{0.7f, 0.7f, 0.7f};
        case 1:
            return Color3{0.8f, 0.8f, 0.8f};
        case 2:
            return Color3{0.45f, 0.7f, 0.95f};
        case 3:
            return Color3{0.95f, 0.75f, 0.35f};
        case 4:
            return Color3{0.55f, 0.9f, 0.55f};
        default:
            return Color3{0.7f, 0.7f, 0.7f};
        }
    }

    void refresh_controls()
    {
        debug_overlay().controls("Controls", std::vector<std::string>{
                                                 "WASD  move camera",
                                                 "Space / Left Shift  up / down",
                                                 "Mouse  look",
                                                 "Esc  release or capture mouse",
                                                 "LMB  select object",
                                                 std::string{"Scroll  spawn shape: "} +
                                                     spawn_shape_name(M_state.spawn_shape_index),
                                                 "RMB  spawn selected shape",
                                                 "Enter  release stasis objects",
                                                 "F  impulse selected released object",
                                                 "Delete  delete selected object",
                                                 "G  toggle gravity",
                                                 "R  reset velocities",
                                                 "F9  debug overlay",
                                             });
    }

    void select_spawn_shape(int direction)
    {
        auto next = static_cast<int>(M_state.spawn_shape_index) + direction;
        if (next < 0)
            next = static_cast<int>(spawn_shape_count) - 1;
        else if (next >= static_cast<int>(spawn_shape_count))
            next = 0;

        M_state.spawn_shape_index = static_cast<unsigned>(next);
        refresh_controls();
    }

    [[nodiscard]] bool is_frozen(world_base::handle handle) const
    {
        for (const auto frozen : M_state.frozen)
            if (frozen == handle) return true;
        return false;
    }

    void remember_frozen(world_base::handle handle) { M_state.frozen.push_back(handle); }

    void forget_frozen(world_base::handle handle)
    {
        for (auto it = M_state.frozen.begin(); it != M_state.frozen.end(); ++it)
        {
            if (*it == handle)
            {
                M_state.frozen.erase(it);
                return;
            }
        }
    }

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
    task<> wait_for_stasis_release(std::shared_ptr<bool> released)
    {
        co_await wait_until_key_press(Key::Enter);
        *released = true;
    }

    task<> spawn_selected_shape(vec3<si::metre> pos, unsigned shape_index)
    {
        auto object = co_await add_rigid(object_desc::dynam()
                                             .with_shape(spawn_shape(shape_index))
                                             .with_pos(pos)
                                             .with_mass(1.0 * kg)
                                             .with_restitution(0.5)
                                             .with_friction(0.5),
                                         spawn_shape_color(shape_index));
        if (!object) co_return;

        auto handle = (*object)->handle();
        auto released = std::make_shared<bool>(false);
        remember_frozen(handle);
        co_await add_task<policy::no_wait>(wait_for_stasis_release(released));

        while (!*released)
        {
            auto obj = co_await get_rigid(handle);
            if (!obj)
            {
                forget_frozen(handle);
                co_return;
            }

            (*obj)->pos() = pos;
            (*obj)->vel() = vec3<si::metre / si::second>::zero();
            (*obj)->orientation(quat<one>::identity());
            (*obj)->ang_vel() = vec3<si::radian / si::second>::zero();

            co_await next_physics_tick{};

            obj = co_await get_rigid(handle);
            if (!obj)
            {
                forget_frozen(handle);
                co_return;
            }

            (*obj)->pos() = pos;
            (*obj)->vel() = vec3<si::metre / si::second>::zero();
            (*obj)->orientation(quat<one>::identity());
            (*obj)->ang_vel() = vec3<si::radian / si::second>::zero();

            co_await next_frame{};
        }

        forget_frozen(handle);
    }

    /// @brief keyboard and mouse-bindings for the user to manipulate objects in the arena.
    task<> maybe_spawn_objects()
    {
        if (get_mouse_button(Pointer::MouseRight).is_initial_press())
        {
            auto spawn_pos = cam().pos() + cam().forward() * 2.0f * m;
            co_await add_task<policy::no_wait>(
                spawn_selected_shape(spawn_pos, M_state.spawn_shape_index));
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
            forget_frozen(selected);
            remove_physics_object(selected);
            M_state.selected.reset();
            co_return;
        }

        // impulse forward
        if (get_key(Key::F).is_initial_press() && obj.is_dynamic() && !is_frozen(selected))
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
        }
    }
};

MAGNUM_APPLICATION_MAIN(sandbox) // NOLINT
