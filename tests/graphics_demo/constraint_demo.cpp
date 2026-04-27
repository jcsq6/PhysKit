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

using namespace graphics;
using namespace physkit;
using namespace mp_units;
using namespace mp_units::si::unit_symbols;

class app : public graphics_app
{
public:
    explicit app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .window_size({1280, 720})
                           .title("Constraints Demo")
                           .cam_pos(fvec3{5.0f, 15.0f, -50.0f} * si::metre)
                           .look_at(fvec3{5.0f, 15.0f, 0.0f} * si::metre)
                           .gravity({0.0 * m / s / s, -9.81 * m / s / s, 0.0 * m / s / s})
                           .solver_iterations(16)}
    // NOLINT
    {
        auto box_mesh = box(vec3<si::metre>{1 * m, 1 * m, 1 * m});
        auto anchor_mesh = box(vec3<si::metre>{0.5 * m, 0.5 * m, 0.5 * m});
        auto long_box = box(vec3<si::metre>{.5 * m, 2 * m, .5 * m});

        auto &w = dynamic_cast<physkit::world &>(world());

        auto floor = w.create_rigid(object_desc::stat()
                                        .with_pos({0 * m, -5 * m, 0 * m})
                                        .with_shape(box(vec3<si::metre>{50 * m, 2 * m, 50 * m})));
        add_object(floor, {0.3f, 0.3f, 0.3f});

        // 1. Distance Constraint (Pendulum)
        {
            auto h1 = w.create_rigid(
                object_desc::stat().with_pos({-12 * m, 15 * m, 0 * m}).with_shape(anchor_mesh));
            auto h2 = w.create_rigid(object_desc::dynam()
                                         .with_pos({-7 * m, 15 * m, 0 * m})
                                         .with_mass(1 * kg)
                                         .with_vel(vec3{0, 2, 2} * m / s)
                                         .with_shape(box_mesh));
            add_object(h1, {0.8f, 0.2f, 0.2f});
            add_object(h2, {1.0f, 0.4f, 0.4f});

            w.add_constraint(impulse::distance_constraint::desc::make(h1, h2)
                                 .with_local_anchor_a({0 * m, 0 * m, 0 * m})
                                 .with_local_anchor_b({0 * m, 0 * m, 0 * m})
                                 .with_distance(4 * m));
        }

        // 2. Ball Socket Constraint (Pendulum)
        {
            auto h1 = w.create_rigid(
                object_desc::stat().with_pos({-4 * m, 15.5 * m, 0 * m}).with_shape(anchor_mesh));
            auto h2 = w.create_rigid(object_desc::dynam()
                                         .with_pos({-4 * m, 12.75 * m, 0 * m})
                                         .with_mass(1 * kg)
                                         .with_ang_vel(vec3{0, 6, 1} * rad / s)
                                         .with_shape(long_box));
            // .with_orientation(quat<one>::from_angle_axis(90.0 * deg, vec3{0, 0, 1})));
            add_object(h1, {0.2f, 0.8f, 0.2f});
            add_object(h2, {0.4f, 1.0f, 0.4f});

            w.add_constraint(impulse::ball_socket_constraint::desc::make(h1, h2).with_anchor(
                {-4 * m, 14.5 * m, 0 * m}));
        }

        // 3. Hinge Constraint (Trapdoor / Swing)
        {
            auto h1 = w.create_rigid(
                object_desc::stat().with_pos({4 * m, 15 * m, 0 * m}).with_shape(anchor_mesh));
            auto h2 = w.create_rigid(object_desc::dynam()
                                         .with_pos({8 * m, 15 * m, 0 * m})
                                         .with_mass(1 * kg)
                                         .with_shape(long_box));
            add_object(h1, {0.2f, 0.2f, 0.8f});
            add_object(h2, {0.4f, 0.4f, 1.0f});

            w.add_constraint(impulse::hinge_constraint::desc::make(h1, h2)
                                 .with_local_anchor_a({0 * m, 0 * m, 0 * m})
                                 .with_local_anchor_b({-4 * m, 0 * m, 0 * m})
                                 .with_local_axis_a({0, 0, 1})
                                 .with_local_axis_b({0, 0, 1}));
        }

        // 4. Slider Constraint (Elevator/Diagonal Slide)
        {
            auto h1 = w.create_rigid(
                object_desc::stat().with_pos({12 * m, 15 * m, 0 * m}).with_shape(anchor_mesh));
            auto h2 = w.create_rigid(object_desc::dynam()
                                         .with_pos({12 * m, 15 * m, 0 * m})
                                         .with_mass(1 * kg)
                                         .with_shape(box_mesh));
            add_object(h1, {0.8f, 0.8f, 0.2f});
            add_object(h2, {1.0f, 1.0f, 0.4f});

            w.add_constraint(impulse::slider_constraint::desc::make(h1, h2)
                                 .with_anchor({12 * m, 15 * m, 0 * m}) // world anchor
                                 .with_axis({1, -2, 0}));              // world axis
        }

        // 5. Weld Constraint (Falling glued pair)
        {
            auto h1 =
                w.create_rigid(object_desc::dynam()
                                   .with_pos({20 * m, 15 * m, 0 * m})
                                   .with_mass(1 * kg)
                                   .with_shape(anchor_mesh)
                                   .with_ang_vel({2.0 * rad / s, 1.0 * rad / s, 0.5 * rad / s})
                                   .with_restitution(.5));
            auto h2 = w.create_rigid(object_desc::dynam()
                                         .with_pos({22 * m, 15 * m, 1 * m})
                                         .with_mass(1 * kg)
                                         .with_shape(box_mesh)
                                         .with_restitution(.5));
            add_object(h1, {0.8f, 0.2f, 0.8f});
            add_object(h2, {1.0f, 0.4f, 1.0f});

            w.add_constraint(impulse::weld_constraint::desc::make(h1, h2).with_anchor(
                {20.5 * m, 15 * m, 0.5 * m})); // world anchor
        }
    }

    void update(quantity<si::second> dt) override
    {
        // update gets called per frame by stepper
    }
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
