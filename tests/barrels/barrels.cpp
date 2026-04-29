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

class barrels_app : public graphics_app
{
    static inline const auto gravity = vec3{0.0, -9.81, 0.0} * m / s / s;
    using typed_world = physkit::world;

public:
    explicit barrels_app(const Platform::Application::Arguments &arguments)
        : graphics_app{g_config(arguments, false)
                           .title("PhysKit Barrels Demo")
                           .window_size({1280, 720})
                           .cam_pos(fvec3{0.0f, 2.0f, -4.0f} * si::metre)
                           .look_at(fvec3{0.0f, 2.0f, 1.5f} * si::metre)
                           .drag(false)
                           .gravity(gravity/2)
                           .time_step(1.0 / 1200.0 * si::second)
                           .solver_iterations(20)}
    {
        cam().speed(1.0f * si::metre / si::second);
        world().add_task(scene());
    }

    void update(mp_units::quantity<mp_units::si::second> dt) override {}

private:
    task<> add_beam(vec3<si::metre> pos, quantity<si::metre> length,
        quantity<si::metre> width,
        float ang,
        Color3 color)
    {
        auto bx = box(vec3<si::metre>{2*length,2*width/5,2*width});

        quat<one> rot = quat<one>::from_angle_axis(ang*deg, vec3<one>{0,0,1});

        co_await add_rigid(object_desc::stat()
                                    .with_shape(std::move(bx))
                                    .with_mass(100*kg)
                                    .with_pos(pos)
                                    .with_orientation(rot)
                                    .with_restitution(0.5)
                                    .with_friction(0.8),
                                    color);
    }
    task <world_base::handle> make_barrel(vec3<si::metre> pos,
        quantity<si::metre> radius,
        quantity<si::metre> height,
        Color3 color)
    {
        auto mesh = cylinder(radius, height);
        auto rot = quat<one>::from_angle_axis(90.0*deg, vec3<one>{1,0,0});

        auto h = (*co_await add_rigid(object_desc::dynam()
                               .with_shape(mesh)
                               .with_pos(pos)
                               .with_mass(100*kg)
                               .with_orientation(rot)
                               .with_restitution(0.5)
                               .with_friction(0.8),
                           color))->handle();
        co_return h;
    }

    task<> scene()
    {
        Color3 red{178.0/255, 34.0/255, 34.0/255};
        Color3 rust{183.0/255,65.0/255,14.0/255};

        auto start_h = 2.0;
        auto spacing = 1.2;
        auto len = 0.5*m;
        auto xdist = 0.5;
        auto wid = 0.1*m;
        auto angle = 5.0;
        for (int i = 0; i < 3; i++)
        {
            auto pos1 = vec3{xdist, start_h-spacing*i, 0}*m;
            auto pos2 = vec3{-xdist, start_h-spacing*i-spacing/2.0, 0}*m;
            //right
            co_await add_beam(pos1, len, wid, angle, red);
            //left
            co_await add_beam(pos2, len, wid, -angle, red);
        }
        //auto mesh = box(vec3{0.1, 0.1, 0.1} * m);
        while (true)
		  {
            auto frame_time = *co_await next_render_frame();
            if (get_mouse_button(Pointer::MouseLeft).is_initial_press())
            {
                co_await make_barrel(vec3{1, 2.4, 0}*m, 0.1*m, 0.1*m, rust);
            }
		  }
    }

};

MAGNUM_APPLICATION_MAIN(barrels_app) // NOLINT
