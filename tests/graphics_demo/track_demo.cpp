#include "graphics.h"
#include <mp-units/systems/si/units.h>

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics::graphics_app{arguments,           {800, 600},           45 * si::degree,
                                 "Camera Track Test", {0.0f, 8.0f, -18.0f}, {0.0f, -8.0f, 18.0f}}
    // NOLINT
    {
        using namespace si::unit_symbols;

        auto sphere = mesh_objs::sphere();

        const physkit::vec3<m, float> pt_a = {-8 * m, 0 * m, -8 * m};
        const physkit::vec3<m, float> pt_b = {8 * m, 0 * m, 8 * m};
        const physkit::vec3<m, float> pt_c = {8 * m, 0 * m, -8 * m};
        const physkit::vec3<m, float> pt_d = {-8 * m, 0 * m, 8 * m};

        // Center marker: small white sphere at origin
        M_center = add_object(sphere, {0.9f, 0.9f, 0.9f});

        M_a = add_object(sphere, {1.0f, 0.15f, 0.1f});
        M_a->translate(to_magnum_vector<float>(pt_a));

        M_b = add_object(sphere, {0.1f, 0.8f, 0.15f});
        M_b->translate(to_magnum_vector<float>(pt_b));

        // Blue cone at +Z
        M_c = add_object(sphere, {0.1f, 0.2f, 1.0f});
        M_c->translate(to_magnum_vector<float>(pt_c));

        // Yellow cylinder at -Z, elevated
        M_d = add_object(sphere, {0.9f, 0.8f, 0.1f});
        M_d->translate(to_magnum_vector<float>(pt_d));

        cam().set_move_track(
            camera_track({
                             kf::make_pos({0 * m, 8 * m, -18 * m}).transition(2.0f * s),
                             kf::make_pos({0 * m, 20 * m, 0 * m}).transition(4.0f * s),
                             kf::make_pos(pt_a + physkit::vec3<m, float>{0 * m, 20 * m, 0 * m})
                                 .transition(4.0f * s),
                             kf::make_pos(pt_b + physkit::vec3<m, float>{0 * m, 20 * m, 0 * m})
                                 .transition(4.0f * s),
                             kf::make_pos(pt_c + physkit::vec3<m, float>{0 * m, 20 * m, 0 * m})
                                 .transition(4.0f * s),
                             kf::make_pos(pt_d + physkit::vec3<m, float>{0 * m, 20 * m, 0 * m})
                                 .transition(4.0f * s),
                             kf::make_pos({0 * m, 20 * m, 0 * m}).transition(4.0f * s),
                         })
                .with_interp(camera_track::spline)
                .with_extrap(camera_track::release));
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        using namespace si::unit_symbols;

        // Spin objects so it is visually obvious the scene is live
        // constexpr auto spin = std::numbers::pi_v<float> * rad / s;
        // M_red_sphere->rotate(spin * dt, {0.0f, 1.0f, 0.0f});
        // M_green_cube->rotate(spin * dt, {1.0f, 1.0f, 0.0f});
    }

    void pointer_move(PointerMoveEvent &event) override { cam().pointer_move(event, false); }

private:
    gfx_obj *M_center{};
    gfx_obj *M_a{};
    gfx_obj *M_b{};
    gfx_obj *M_c{};
    gfx_obj *M_d{};
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
