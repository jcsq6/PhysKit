#include "graphics.h"
#include <mp-units/systems/si/units.h>
#include <cmath>

using namespace graphics;
using namespace physkit;

class app : public graphics::graphics_app
{
public:
    explicit app(const Arguments &arguments)
        : graphics::graphics_app{arguments,
                                 {1280, 720},
                                 45 * si::degree,
                                 "Solar System",
                                 {0.0f, 30.0f, -50.0f},
                                 {0.0f, -30.0f, 50.0f},
                                 true}
    // NOLINT
    {
        using namespace si::unit_symbols;

        auto sphere = mesh_objs::sphere();
        auto cyl = mesh_objs::cylinder(1, 48, 0.5f, false);

        // Sun
        M_sun = add_object(sphere, {1.0f, 0.85f, 0.2f});
        M_sun->scale({3.0f, 3.0f, 3.0f});

        // Mercury
        M_mercury = add_object(sphere, {0.6f, 0.6f, 0.6f});
        M_mercury->scale({0.3f, 0.3f, 0.3f});

        // Venus
        M_venus = add_object(sphere, {0.9f, 0.8f, 0.5f});
        M_venus->scale({0.55f, 0.55f, 0.55f});

        // Earth
        M_earth = add_object(sphere, {0.2f, 0.4f, 0.9f});
        M_earth->scale({0.6f, 0.6f, 0.6f});

        // Moon
        M_moon = add_object(sphere, {0.75f, 0.75f, 0.75f});
        M_moon->scale({0.18f, 0.18f, 0.18f});

        // Mars
        M_mars = add_object(sphere, {0.85f, 0.3f, 0.15f});
        M_mars->scale({0.4f, 0.4f, 0.4f});

        // Jupiter
        M_jupiter = add_object(sphere, {0.8f, 0.6f, 0.35f});
        M_jupiter->scale({1.8f, 1.8f, 1.8f});

        // Saturn
        M_saturn = add_object(sphere, {0.85f, 0.75f, 0.4f});
        M_saturn->scale({1.5f, 1.5f, 1.5f});

        // Saturn ring
        M_saturn_ring = add_object(cyl, {0.75f, 0.65f, 0.35f});

        // Uranus
        M_uranus = add_object(sphere, {0.5f, 0.8f, 0.85f});
        M_uranus->scale({0.9f, 0.9f, 0.9f});

        // Neptune
        M_neptune = add_object(sphere, {0.2f, 0.3f, 0.9f});
        M_neptune->scale({0.85f, 0.85f, 0.85f});

        // Camera track: tour the solar system
        const auto origin = physkit::vec3<m, float>{0 * m, 0 * m, 0 * m};

        cam().set_move_track(
            camera_track({
                             // High overview
                             kf::make_pos({0 * m, 35 * m, -45 * m}).look_at(origin).transition(4.0f * s),
                             // Swoop down toward inner planets
                             kf::make_pos({8 * m, 8 * m, -12 * m}).look_at(origin).transition(3.0f * s),
                             // Pass Earth orbit
                             kf::make_pos({12 * m, 5 * m, 5 * m}).look_at(origin).transition(3.0f * s),
                             // Out toward Mars
                             kf::make_pos({5 * m, 6 * m, 14 * m}).look_at(origin).transition(3.0f * s),
                             // Jupiter flyby
                             kf::make_pos({-10 * m, 8 * m, 18 * m}).look_at(origin).transition(3.0f * s),
                             // Saturn region
                             kf::make_pos({-22 * m, 10 * m, 8 * m}).look_at(origin).transition(3.0f * s),
                             // Outer planets sweep
                             kf::make_pos({-20 * m, 15 * m, -15 * m}).look_at(origin).transition(3.0f * s),
                             // High vantage point
                             kf::make_pos({0 * m, 45 * m, 0 * m}).look_at(origin).transition(4.0f * s),
                             // Swing back down
                             kf::make_pos({15 * m, 20 * m, -30 * m}).look_at(origin).transition(3.5f * s),
                             // Return to start
                             kf::make_pos({0 * m, 35 * m, -45 * m}).look_at(origin).transition(0.0f * s),
                         },
                         camera_track::spline));
    }

    void update(physkit::quantity<physkit::si::second> dt) override
    {
        using namespace si::unit_symbols;
        M_elapsed += dt;
        const float t = static_cast<double>(M_elapsed.numerical_value_in(s));

        // Orbital speeds (rad/s)
        constexpr float spd_mercury = 1.8f;
        constexpr float spd_venus = 1.3f;
        constexpr float spd_earth = 1.0f;
        constexpr float spd_mars = 0.75f;
        constexpr float spd_jupiter = 0.4f;
        constexpr float spd_saturn = 0.28f;
        constexpr float spd_uranus = 0.18f;
        constexpr float spd_neptune = 0.12f;
        constexpr float spd_moon = 4.0f;

        // Orbit radii
        constexpr float r_mercury = 5.0f;
        constexpr float r_venus = 7.0f;
        constexpr float r_earth = 9.5f;
        constexpr float r_mars = 12.0f;
        constexpr float r_jupiter = 16.0f;
        constexpr float r_saturn = 20.0f;
        constexpr float r_uranus = 24.0f;
        constexpr float r_neptune = 28.0f;
        constexpr float r_moon = 1.5f;

        // Self-rotation speed
        constexpr float spin = 50.0f; // degrees per second

        // Sun: rotate in place
        M_sun->resetTransformation()
            .scale({3.0f, 3.0f, 3.0f})
            .rotateY(Deg(t * spin * 0.2f));

        // Mercury
        const float mercury_x = r_mercury * std::cos(spd_mercury * t);
        const float mercury_z = r_mercury * std::sin(spd_mercury * t);
        M_mercury->resetTransformation()
            .scale({0.3f, 0.3f, 0.3f})
            .rotateY(Deg(t * spin))
            .translate({mercury_x, 0.0f, mercury_z});

        // Venus
        const float venus_x = r_venus * std::cos(spd_venus * t);
        const float venus_z = r_venus * std::sin(spd_venus * t);
        M_venus->resetTransformation()
            .scale({0.55f, 0.55f, 0.55f})
            .rotateY(Deg(t * spin * 0.9f))
            .translate({venus_x, 0.0f, venus_z});

        // Earth
        const float earth_x = r_earth * std::cos(spd_earth * t);
        const float earth_z = r_earth * std::sin(spd_earth * t);
        M_earth->resetTransformation()
            .scale({0.6f, 0.6f, 0.6f})
            .rotateY(Deg(t * spin * 1.5f))
            .translate({earth_x, 0.0f, earth_z});

        // Moon: orbits Earth
        const float moon_x = earth_x + (r_moon * std::cos(spd_moon * t));
        const float moon_z = earth_z + (r_moon * std::sin(spd_moon * t));
        M_moon->resetTransformation()
            .scale({0.18f, 0.18f, 0.18f})
            .translate({moon_x, 0.0f, moon_z});

        // Mars
        const float mars_x = r_mars * std::cos(spd_mars * t);
        const float mars_z = r_mars * std::sin(spd_mars * t);
        M_mars->resetTransformation()
            .scale({0.4f, 0.4f, 0.4f})
            .rotateY(Deg(t * spin * 1.2f))
            .translate({mars_x, 0.0f, mars_z});

        // Jupiter
        const float jupiter_x = r_jupiter * std::cos(spd_jupiter * t);
        const float jupiter_z = r_jupiter * std::sin(spd_jupiter * t);
        M_jupiter->resetTransformation()
            .scale({1.8f, 1.8f, 1.8f})
            .rotateY(Deg(t * spin * 2.0f))
            .translate({jupiter_x, 0.0f, jupiter_z});

        // Saturn
        const float saturn_x = r_saturn * std::cos(spd_saturn * t);
        const float saturn_z = r_saturn * std::sin(spd_saturn * t);
        M_saturn->resetTransformation()
            .scale({1.5f, 1.5f, 1.5f})
            .rotateY(Deg(t * spin * 1.8f))
            .translate({saturn_x, 0.0f, saturn_z});

        // Saturn ring: flat disk that follows Saturn, tilted slightly
        M_saturn_ring->resetTransformation()
            .scale({3.5f, 0.02f, 3.5f})
            .rotateX(Deg(15.0f))
            .translate({saturn_x, 0.0f, saturn_z});

        // Uranus
        const float uranus_x = r_uranus * std::cos(spd_uranus * t);
        const float uranus_z = r_uranus * std::sin(spd_uranus * t);
        M_uranus->resetTransformation()
            .scale({0.9f, 0.9f, 0.9f})
            .rotateY(Deg(t * spin * 1.1f))
            .translate({uranus_x, 0.0f, uranus_z});

        // Neptune
        const float neptune_x = r_neptune * std::cos(spd_neptune * t);
        const float neptune_z = r_neptune * std::sin(spd_neptune * t);
        M_neptune->resetTransformation()
            .scale({0.85f, 0.85f, 0.85f})
            .rotateY(Deg(t * spin * 1.3f))
            .translate({neptune_x, 0.0f, neptune_z});
    }

private:
    physkit::quantity<physkit::si::second> M_elapsed{};
    gfx_obj *M_sun{};
    gfx_obj *M_mercury{};
    gfx_obj *M_venus{};
    gfx_obj *M_earth{};
    gfx_obj *M_moon{};
    gfx_obj *M_mars{};
    gfx_obj *M_jupiter{};
    gfx_obj *M_saturn{};
    gfx_obj *M_saturn_ring{};
    gfx_obj *M_uranus{};
    gfx_obj *M_neptune{};
};

MAGNUM_APPLICATION_MAIN(app) // NOLINT
