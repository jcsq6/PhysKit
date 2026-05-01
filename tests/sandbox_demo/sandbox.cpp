// creating a physics sandbox for contact visualizer

#ifdef PHYSKIT_GRAPHICS_MODULES
#include <Magnum/Platform/GlfwApplication.h>
#endif

#ifndef PHYSKIT_IMPORT_STD
#include <algorithm>
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
    struct grab_state
    {
        world_base::handle handle;
        quantity<si::metre> distance{};
        vec3<si::metre> local_point{};
        vec3<si::metre> previous_target{};
    };

    struct stasis_impulse_buffer
    {
        vec3<si::kilogram * si::metre / si::second> linear{};
        vec3<si::kilogram * si::metre * si::metre / si::second> angular{};
    };

    std::optional<world_base::handle> selected;
    std::optional<grab_state> grabbed;
    std::vector<world_base::handle> frozen;
    std::vector<world_base::handle> spawned;
    unsigned spawn_shape_index = 0;
    double spawn_yaw_degrees = 0.0;
    bool gravity_on = true;
    double gravity_magnitude = 9.81;
    vec3<si::metre / si::second / si::second> saved_gravity{};
};

class sandbox : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    static inline const auto grab_response_time = 0.06 * s;
    static inline const auto max_grab_speed = 80.0 * m / s;
    static inline const auto stasis_click_impulse_speed = 10.0 * m / s;
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
                                                 "LMB  grab object / charge stasis impulse",
                                                 std::string{"Scroll  spawn shape: "} +
                                                     spawn_shape_name(M_state.spawn_shape_index),
                                                 "RMB  spawn shape at crosshair surface",
                                                 "Q / E  rotate spawn preview",
                                                 "C  duplicate selected object",
                                                 "Enter  release stasis objects",
                                                 "Delete  delete selected object",
                                                 "G  toggle gravity",
                                                 "- / =  decrease / increase gravity",
                                                 "P  pause / play physics",
                                                 "N  single physics step while paused",
                                                 "R  reset velocities",
                                                 "X  clear dynamic scene",
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

    [[nodiscard]] quat<one> spawn_orientation() const
    {
        return quat<one>::from_angle_axis(M_state.spawn_yaw_degrees * deg,
                                          vec3<one>{0.0, 1.0, 0.0});
    }

    void set_gravity_from_state()
    {
        M_state.saved_gravity = vec3{0.0, -M_state.gravity_magnitude, 0.0} * m / s / s;
        mutable_world_gravity() =
            M_state.gravity_on ? M_state.saved_gravity : vec3{0.0, 0.0, 0.0} * m / s / s;
    }

    void change_gravity(double delta)
    {
        M_state.gravity_magnitude = std::max(0.0, M_state.gravity_magnitude + delta);
        set_gravity_from_state();
        refresh_controls();
    }

    [[nodiscard]] vec3<si::metre> placement_offset(const shape &shp) const
    {
        const auto bounds = shp.at(vec3<si::metre>::zero(), spawn_orientation()).bounds();
        return vec3{0.0 * m, -bounds.min.y() + 0.03 * m, 0.0 * m};
    }

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
    enum class stasis_input
    {
        left_click,
        release,
    };

    struct stasis_input_callbacks
    {
        callback_id left_click{};
        callback_id release{};
    };

    auto wait_until_stasis_input(stasis_input &input)
    {
        return physkit::wait_for_event{
            .setup_fn =
                [this, &input](auto resume)
            {
                const auto click_resume = resume;
                const auto release_resume = resume;
                auto did_resume = std::make_shared<bool>(false);

                return stasis_input_callbacks{
                    .left_click = on_click(
                        [click_resume, &input, did_resume](PointerEvent &event)
                        {
                            if (event.pointer() == Pointer::MouseLeft)
                            {
                                if (*did_resume) return true;
                                *did_resume = true;
                                input = stasis_input::left_click;
                                click_resume();
                                return true;
                            }
                            return false;
                        }),
                    .release = on_key_press(
                        [release_resume, &input, did_resume](KeyEvent &event)
                        {
                            if (event.key() == Key::Enter)
                            {
                                input = stasis_input::release;
                                if (*did_resume) return true;
                                *did_resume = true;
                                release_resume();
                                return true;
                            }
                            return false;
                        }),
                };
            },
            .destroy_fn =
                [this](stasis_input_callbacks callbacks)
            {
                remove_click(callbacks.left_click);
                remove_key_press(callbacks.release);
            }};
    }

    task<> buffer_stasis_impulse_if_hit(
        world_base::handle handle,
        const std::shared_ptr<sandbox_state::stasis_impulse_buffer> &buffer)
    {
        auto ray = physkit::ray{cam().pos(), cam().forward()};
        auto hits = co_await raycast{.r = ray};

        for (auto [hit_handle, distance] : hits)
        {
            auto obj_opt = co_await get_rigid(hit_handle);
            if (!obj_opt || !(*obj_opt)->is_dynamic()) continue;
            if (hit_handle != handle) co_return;

            auto &obj = **obj_opt;
            const auto impulse = ray.direction() * (obj.mass() * stasis_click_impulse_speed);
            const auto hit_pos = ray.origin() + ray.direction() * distance;

            buffer->linear += impulse;
            buffer->angular += (hit_pos - obj.pos()).cross(impulse);
            co_return;
        }
    }

    task<> watch_stasis_input(world_base::handle handle, std::shared_ptr<bool> released,
                              std::shared_ptr<sandbox_state::stasis_impulse_buffer> buffer)
    {
        while (!*released)
        {
            auto input = stasis_input::left_click;
            co_await wait_until_stasis_input(input);

            if (input == stasis_input::release)
            {
                *released = true;
                co_return;
            }

            co_await buffer_stasis_impulse_if_hit(handle, buffer);
        }
    }

    task<> spawn_selected_shape(vec3<si::metre> pos, unsigned shape_index)
    {
        const auto orientation = spawn_orientation();
        auto object = co_await add_rigid(object_desc::dynam()
                                             .with_shape(spawn_shape(shape_index))
                                             .with_pos(pos)
                                             .with_orientation(orientation)
                                             .with_mass(1.0 * kg)
                                             .with_restitution(0.5)
                                             .with_friction(0.5),
                                         spawn_shape_color(shape_index));
        if (!object) co_return;

        auto handle = (*object)->handle();
        M_state.spawned.push_back(handle);
        auto released = std::make_shared<bool>(false);
        auto impulse_buffer = std::make_shared<sandbox_state::stasis_impulse_buffer>();
        remember_frozen(handle);
        auto input_task = co_await add_task{watch_stasis_input(handle, released, impulse_buffer)};
        if (!input_task)
        {
            forget_frozen(handle);
            co_return;
        }

        while (!*released)
        {
            auto obj = co_await get_rigid(handle);
            if (!obj)
            {
                *released = true;
                forget_frozen(handle);
                co_await cancel_task{*input_task};
                co_return;
            }

            (*obj)->pos() = pos;
            (*obj)->vel() = vec3<si::metre / si::second>::zero();
            (*obj)->orientation(orientation);
            (*obj)->ang_vel() = vec3<si::radian / si::second>::zero();

            co_await next_physics_tick{};

            obj = co_await get_rigid(handle);
            if (!obj)
            {
                *released = true;
                forget_frozen(handle);
                co_await cancel_task{*input_task};
                co_return;
            }

            (*obj)->pos() = pos;
            (*obj)->vel() = vec3<si::metre / si::second>::zero();
            (*obj)->orientation(orientation);
            (*obj)->ang_vel() = vec3<si::radian / si::second>::zero();

            co_await next_frame{};
        }

        forget_frozen(handle);

        auto obj = co_await get_rigid(handle);
        if (obj && (*obj)->is_dynamic())
        {
            (*obj)->apply_impulse(impulse_buffer->linear);
            (*obj)->apply_angular_impulse(impulse_buffer->angular);
        }

        co_await cancel_task{*input_task};
    }

    task<vec3<si::metre>> spawn_position_for_crosshair(unsigned shape_index)
    {
        const auto shp = spawn_shape(shape_index);
        auto ray = physkit::ray{cam().pos(), cam().forward()};
        auto hits = co_await raycast{.r = ray};

        for (auto [handle, distance] : hits)
        {
            auto obj_opt = co_await get_rigid(handle);
            if (!obj_opt || is_frozen(handle)) continue;
            co_return ray.origin() + ray.direction() * distance + placement_offset(shp);
        }

        co_return cam().pos() + cam().forward() * 2.0f * m;
    }

    task<> duplicate_selected()
    {
        if (!M_state.selected) co_return;

        auto source = co_await get_rigid(*M_state.selected);
        if (!source || !(*source)->is_dynamic()) co_return;

        auto &obj = **source;
        auto pos = obj.pos() + cam().right() * 0.75f * m;
        auto object = co_await add_rigid(object_desc::dynam()
                                             .with_shape(obj.shape())
                                             .with_pos(pos)
                                             .with_orientation(obj.orientation())
                                             .with_mass(obj.mass())
                                             .with_restitution(obj.restitution())
                                             .with_friction(obj.friction()),
                                         Color3{0.85f, 0.85f, 0.95f});
        if (!object) co_return;

        M_state.spawned.push_back((*object)->handle());
        M_state.selected = (*object)->handle();
    }

    /// @brief keyboard and mouse-bindings for the user to manipulate objects in the arena.
    task<> maybe_spawn_objects()
    {
        if (get_mouse_button(Pointer::MouseRight).is_initial_press())
        {
            auto spawn_pos = *co_await spawn_position_for_crosshair(M_state.spawn_shape_index);
            co_await add_task<policy::no_wait>(
                spawn_selected_shape(spawn_pos, M_state.spawn_shape_index));
        }
    }

    [[nodiscard]] vec3<si::metre> grab_target(const sandbox_state::grab_state &grab)
    { return cam().pos() + cam().forward() * grab.distance; }

    task<> begin_grab()
    {
        auto ray = physkit::ray{cam().pos(), cam().forward()};
        auto hits = co_await raycast{.r = ray};
        M_state.selected.reset();
        M_state.grabbed.reset();

        for (auto [handle, distance] : hits)
        {
            auto obj_opt = co_await get_rigid(handle);
            if (obj_opt && (*obj_opt)->is_dynamic())
            {
                M_state.selected = handle;
                if (!is_frozen(handle))
                {
                    auto hit_pos = ray.origin() + ray.direction() * distance;
                    M_state.grabbed = sandbox_state::grab_state{
                        .handle = handle,
                        .distance = distance,
                        .local_point = (*obj_opt)->project_to_local(hit_pos),
                        .previous_target = hit_pos,
                    };
                }
                break;
            }
        }
    }

    task<> handle_grab_input(quantity<si::second> frame_dt)
    {
        const auto left_mouse = get_mouse_button(Pointer::MouseLeft);
        if (left_mouse.is_initial_press()) co_await begin_grab();

        if (!M_state.grabbed) co_return;
        if (!left_mouse.is_pressed())
        {
            M_state.grabbed.reset();
            co_return;
        }

        auto &grab = *M_state.grabbed;
        auto obj_opt = co_await get_rigid(grab.handle);
        if (!obj_opt || !(*obj_opt)->is_dynamic() || is_frozen(grab.handle))
        {
            M_state.grabbed.reset();
            co_return;
        }

        auto &obj = **obj_opt;
        auto target = grab_target(grab);
        auto target_velocity = vec3<si::metre / si::second>::zero();
        if (frame_dt > 0.0 * s) target_velocity = (target - grab.previous_target) / frame_dt;

        const auto grabbed_point = obj.project_to_world(grab.local_point);
        auto desired_velocity = target_velocity + (target - grabbed_point) / grab_response_time;
        if (auto speed = desired_velocity.norm(); speed > max_grab_speed)
            desired_velocity = desired_velocity / speed * max_grab_speed;

        obj.vel() = desired_velocity;
        obj.ang_vel() *= 0.85 * one;
        grab.previous_target = target;
    }

    /// @brief handles delete and velocity reset
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

        if (get_key(Key::Equal).is_initial_press()) change_gravity(1.0);
        if (get_key(Key::Minus).is_initial_press()) change_gravity(-1.0);

        if (get_key(Key::P).is_initial_press()) physics_paused(!physics_paused());
        if (get_key(Key::N).is_initial_press())
        {
            physics_paused(true);
            step_physics_once();
        }

        if (get_key(Key::Q).is_initial_press())
        {
            M_state.spawn_yaw_degrees -= 15.0;
            refresh_controls();
        }
        if (get_key(Key::E).is_initial_press())
        {
            M_state.spawn_yaw_degrees += 15.0;
            refresh_controls();
        }

        if (get_key(Key::C).is_initial_press()) co_await duplicate_selected();
        if (get_key(Key::X).is_initial_press())
        {
            co_await clear_dynamic_scene();
            co_return;
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
            M_state.grabbed.reset();
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

    task<> clear_dynamic_scene()
    {
        std::vector<world_base::handle> to_remove;
        for (auto *phys_obj : physics_objects())
        {
            auto obj = co_await get_rigid(phys_obj->handle());
            if (obj && (*obj)->is_dynamic()) to_remove.push_back(phys_obj->handle());
        }

        for (auto handle : to_remove)
        {
            co_await destroy_rigid{handle};
            forget_frozen(handle);
            remove_physics_object(handle);
        }

        M_state.selected.reset();
        M_state.grabbed.reset();
        M_state.frozen.clear();
        M_state.spawned.clear();
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
            auto frame_dt = *co_await next_render_frame();
            co_await maybe_spawn_objects();
            co_await handle_grab_input(frame_dt);
            co_await handle_actions_input();
        }
    }
};

MAGNUM_APPLICATION_MAIN(sandbox) // NOLINT
