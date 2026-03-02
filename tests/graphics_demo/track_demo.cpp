#include "graphics.h"
#include <mp-units/systems/si/units.h>

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics::graphics_app{g_config(arguments, false)
                                     .window_size({800, 600})
                                     .title("Camera Track Test")
                                     .cam_pos(fvec3{0.0f, 8.0f, -18.0f} * si::metre)
                                     .cam_dir(fvec3{0.0f, -8.0f, 18.0f} * physkit::one)}
    // NOLINT
    {
        using namespace si::unit_symbols;
        using namespace physkit;

        auto sphere = mesh_objs::sphere();

        const auto pt_a = fvec3{-8, 0, -8} * m;
        const auto pt_b = fvec3{8, 0, 8} * m;
        const auto pt_c = fvec3{8, 0, -8} * m;
        const auto pt_d = fvec3{-8, 0, 8} * m;

        // Center marker: small white sphere at origin
        M_center = add_object(sphere, {0.9f, 0.9f, 0.9f});

        M_a = add_object(sphere, {1.0f, 0.15f, 0.1f});
        M_a->translate(to_magnum_vector<m, float>(pt_a));

        M_b = add_object(sphere, {0.1f, 0.8f, 0.15f});
        M_b->translate(to_magnum_vector<m, float>(pt_b));

        // Blue cone at +Z
        M_c = add_object(sphere, {0.1f, 0.2f, 1.0f});
        M_c->translate(to_magnum_vector<m, float>(pt_c));

        // Yellow cylinder at -Z, elevated
        M_d = add_object(sphere, {0.9f, 0.8f, 0.1f});
        M_d->translate(to_magnum_vector<m, float>(pt_d));

        cam().set_move_track(
            camera_track({
                             kf::make_pos(fvec3{0, 8, -18} * m).transition(2.0f * s),
                             kf::make_pos(fvec3{0, 20, 0} * m).transition(4.0f * s),
                             kf::make_pos(pt_a + fvec3{0, 20, 0} * m).transition(4.0f * s),
                             kf::make_pos(pt_b + fvec3{0, 20, 0} * m).transition(4.0f * s),
                             kf::make_pos(pt_c + fvec3{0, 20, 0} * m).transition(4.0f * s),
                             kf::make_pos(pt_d + fvec3{0, 20, 0} * m).transition(4.0f * s),
                             kf::make_pos(fvec3{0, 20, 0} * m).transition(4.0f * s),
                         })
                .with_interp(camera_track::spline)
                .with_extrap(camera_track::reverse));
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        using namespace si::unit_symbols;
    }

private:
    gfx_obj *M_center{};
    gfx_obj *M_a{};
    gfx_obj *M_b{};
    gfx_obj *M_c{};
    gfx_obj *M_d{};
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
